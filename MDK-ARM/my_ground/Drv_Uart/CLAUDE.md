# Drv_Uart — C 虚函数表（vtable）串口驱动架构

## 概述

用 C 语言虚函数表将串口驱动分层解耦。UART 层**只负责字节搬运**——收发、缓存、硬件句柄访问。不关心上层用什么协议（ANO、SBUS、NMEA 等）。

当前处于**迁移过渡期**：新 vtable 架构与旧 `Drv_Uart.c` 并存，逐步替换。

## 分层架构

```
App/Protocol 层     COM_DT（ANO 协议引擎）、SBUS、GPS 等
                     ↕ uc_uart_read_ucbyte() / uart_transmit() 轮询消费
Board 层             boart_uart_init.c, uarts.h
                     ↕ 实例化 + huart 绑定
Subclass 层          uart_true.c/h（IT/DMA 子类）
                     ↕ container_of 向下转型
Base 层              uart_base.c/h（stUartBase + stUartOps）
```

---

### Base 层 — `uart_base.h / uart_base.c`

定义基类和操作表。

```c
typedef struct stUartBase stUartBase;

typedef struct {
    void (* transmit)(stUartBase *me, uint8_t *pucData, uint16_t usLen);
    void (* receive)(stUartBase *me);
    void (* fifoinit)(stUartBase *me);
    void (* fifoadd)(stUartBase *me, uint16_t usLen);
    UART_HandleTypeDef* (*uart_handle_get)(stUartBase* me);
    osSemaphoreId_t (*Semap_uart_ring_getSemap)(stUartBase* stBase);
    uint8_t (* read_ucbyte)(stUartBase* stBase, uint8_t* pucdata);
    void (* transmit_debug)(stUartBase *me, uint8_t *pucData, uint16_t usLen);
    void (*uart_Ring_tx_complete_isr)(stUartBase* stBase);
} stUartOps;

struct stUartBase { const stUartOps* ops; };
```

**`uart_Ring_tx_complete_isr`** — 新增 vtable 入口。TX 完成中断（`HAL_UART_TxCpltCallback`）通过此函数指针分发到具体端口的 ISR handler（DMA 或 IT 版本）。

**分发函数：**

```c
void uart_transmit(stUartBase* me, uint8_t *pucData, uint16_t usLen);
void uart_receive_enable(stUartBase* me);
void uart_fifo_init(stUartBase* me);
void uart_fifo_add(stUartBase *me, uint16_t usLen);
UART_HandleTypeDef* uart_handle_get(stUartBase* me);
osSemaphoreId_t Semap_uart_ring_getSemap(stUartBase* me);
uint8_t uc_uart_read_ucbyte(stUartBase *me, uint8_t *pucdata);
void uart_Ring_tx_complete_isr(stUartBase* me);
```

---

### Subclass 层 — `uart_true.c/h`

继承 `stUartBase`，扩展私有数据。

```c
typedef struct {
    uint8_t data[64];
    uint16_t len;
} tx_queue_item_t;

typedef struct {
    stUartBase stBase;                          // 基类 — 必须是第一个成员
    UART_HandleTypeDef* pUartHandle;            // HAL UART 句柄
    osSemaphoreId_t   pBinary_SemaphoreHandle;  // TX 完成信号量（IT 模式用）
                                                // 值类型非指针，无需动态创建

    uint8_t*  pucRxbuffer;                      // HAL 接收缓冲区
    uint16_t  usRxbuffer_size;                  // 接收缓冲区大小

    pstRingBufTdf pstdRingbuffer;               // RingBuffer 指针

    QueueHandle_t    xTxQuene_t;                // TX 队列句柄（非阻塞发送）
    volatile uint8_t TxInProgress_vuc;          // 1 = DMA 正在忙
    uint8_t*         TxDmabuffer_puc;           // 指向 .dma_buf 的 persistent TX buffer
} stUartRing;
```

**新增 TX 队列字段说明：**

| 字段 | 作用 |
|------|------|
| `xTxQuene_t` | FreeRTOS 队列（容量 8，元素类型 `tx_queue_item_t`），存储待发送帧副本 |
| `TxInProgress_vuc` | 门卫标志：0=DMA 空闲，1=DMA 正忙。任务级和 ISR 级共享 |
| `TxDmabuffer_puc` | 指向 `.dma_buf` 段的 persistent buffer（non-cacheable），DMA 从此读取 |

**两个 vtable 实例：**

| 实例 | 适用 | transmit | complete_isr | 说明 |
|------|------|----------|-------------|------|
| `usart_ring_it_ops` | IT 模式 | `uart_Ring_tx_IT`（信号量阻塞） | `uart_Ring_tx_complete_isr_IT`（给信号量） | 兼容旧阻塞调用方 |
| `usart_ring_dma_ops` | DMA 模式 | `uart_Ring_tx_DMA_Queue`（非阻塞队列） | `uart_Ring_tx_complete_isr_DMA`（出队续发） | 主用模式，不阻塞 |

**IT ops 也挂在 vtable 中以支持两种路径共存。**

#### 非阻塞 DMA TX 流程

```
任务级 uart_Ring_tx_DMA_Queue():
  1. memcpy 到 tx_queue_item_t（栈上临时变量）
  2. xQueueSend 入队（0 超时，满则丢帧）
  3. TXInProgress==0 → 出队，memcpy 到 TxDmabuffer_puc
                       HAL_UART_Transmit_DMA()
                       TxInProgress=1
  4. 返回（不阻塞）

ISR HAL_UART_TxCpltCallback → uart_Ring_tx_complete_isr_DMA():
  1. xQueueReceiveFromISR 出队到栈上 item
  2. memcpy(me->TxDmabuffer_puc, item.data, item.len)
  3. HAL_UART_Transmit_DMA() 启动下一帧
  4. 队列空 → TxInProgress=0
```

- **IT 模式**同样使用队列非阻塞（但当前 IT 端口如 UART8 只用阻塞发送）
- **cache-safe**: TxDmabuffer_puc 在 `.dma_buf`（non-cacheable），无需 `SCB_CleanDCache_by_Addr`
- **不需要 DMA_HandleTypeDef***: 通过 `me->pUartHandle->hdmarx` 直接访问

#### 接收流程（不变）

```
ISR: HAL_UARTEx_RxEventCallback(huart, Size)
  → uart_fifo_add(): 逐个字节写入 RingBuffer
  → DMA 循环模式不 re-enable（自动连续接收）

任务级: 协议层通过 uc_uart_read_ucbyte() 轮询读取
```

#### 核心函数清单

| 函数 | 可见性 | 职责 |
|------|--------|------|
| `uart_Ring_tx_IT` | static | IT 阻塞发送（信号量同步） |
| `uart_Ring_tx_complete_isr_IT` | static | IT 完成 ISR：给信号量 |
| `uart_Ring_tx_DMA_Queue` | static | DMA 非阻塞发送（入队+启动） |
| `uart_Ring_tx_complete_isr_DMA` | static | DMA 完成 ISR：出队+续发 |
| `uart_Ring_tx_initQuene` | static | 创建 TX 队列，初始化门卫标志 |
| `uart_Ring_rx_DMA` | static | 启用 DMA 循环接收，关闭过半中断 |
| `uart_Ring_rx_IT` | static | 启用 IDLE 中断接收 |
| `uart_ring_add` | static | ISR 中从 DMA buffer 拷贝到 RingBuffer |
| `uart_ring_read` | static | 从 RingBuffer 读一个字节 |
| `Semap_uart_ring_getSemap` | static | 返回 TX 完成信号量句柄 |
| `uart_handle_get` | static | 返回 `UART_HandleTypeDef*` |

#### 初始化函数签名

```c
void uartRing_init_dma(stUartRing *me,
                       UART_HandleTypeDef *pstHandle,
                       uint8_t *DMATxbuffer_puc,   // .dma_buf TX buffer（必须 non-cacheable）
                       uint8_t *pucRxbuffer,
                       uint16_t usRxbuffer_size,
                       pstRingBufTdf pstdRingbuffer,
                       uint8_t *pucRingBuffer,
                       uint32_t ulLen);

void uartRing_init_it(stUartRing *me,
                      UART_HandleTypeDef *pstHandle,
                      uint8_t *pucRxbuffer,
                      uint16_t usRxbuffer_size,
                      pstRingBufTdf pstdRingbuffer,
                      uint8_t *pucRingBuffer,
                      uint32_t ulLen,
                      osSemaphoreId_t pBinary_SemaphoreHandle);
```

**变化对比：**

| 参数 | 旧版 | 新版 |
|------|------|------|
| `DMATxbuffer_puc` | 无 | 新增，指向 `.dma_buf` 的 TX buffer |
| `pDma_rxHandle` | `DMA_HandleTypeDef*` | 移除，通过 `pUartHandle->hdmarx` 访问 |
| `pBinary_SemaphoreHandle` | `osSemaphoreId_t*` (指针) | `osSemaphoreId_t` (值类型) |

---

### Board 层 — `boart_uart_init.c / uarts.h`

唯一分配数组和实例的地方。

```c
// 分配 .dma_buf TX buffer（non-cacheable）
__attribute__((section(".dma_buf"))) static uint8_t COM_TxBuffer[40];
__attribute__((section(".dma_buf"))) static uint8_t LX_TxBuffer[40];

void uart_board_init(void)
{
    uartRing_init_dma(&com_uart, &huart1, COM_TxBuffer, ...);
    uartRing_init_dma(&lx_uart, &huart4, LX_TxBuffer, ...);

    pstbase_com_uart = &com_uart.stBase;
    pstbase_lx_uart  = &lx_uart.stBase;
    // 不在此处开启接收——初始状态由首次 uart_receive_enable() 触发
}
DRIVER_INIT_1(uart_board_init);
```

## 中断回调（`Drv_Uart.c`）

### TX 完成回调

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_handle_get(pstbase_lx_uart))
        uart_Ring_tx_complete_isr(pstbase_lx_uart);
    if (huart == uart_handle_get(pstbase_com_uart))
        uart_Ring_tx_complete_isr(pstbase_com_uart);
}
```

通过 vtable 分派到具体端口：DMA 端口走 `uart_Ring_tx_complete_isr_DMA`（出队续发），IT 端口走 `uart_Ring_tx_complete_isr_IT`（给信号量）。

### 错误恢复回调

错误中断后 `AbortReceive_IT` + 重置接收（`uart_receive_enable`），在 `HAL_UART_AbortCpltCallback` 中完成。所有已迁移端口（USART1、UART4、USART2、UART8）都加了错误恢复。

### 接收事件回调（不变）

DMA 循环模式不 re-enable，ISR 只负责将数据写入 RingBuffer。

## 与 ANO 协议 vtable 的关系

```
ANO 协议层           ← 管"帧语义"
  ↓ vano_check_data() 内部调用 uc_uart_read_ucbyte()
UART 字节层          ← 管"字节搬运"（本层）
```

两层 vtable 完全解耦，UART 层不持有任何协议指针。

## 迁移状态

| 串口 | 子类模式 | 迁移状态 | Board 层 | 备注 |
|------|---------|---------|---------|------|
| USART1 (COM) | `stUartRing` DMA | **已迁移** ✅ | `boart_uart_init.c` | RingBuffer + vtable |
| UART4 (IMU/LX) | `stUartRing` DMA | **已迁移** ✅ | `boart_uart_init.c` | RingBuffer + vtable |
| USART2 (OF) | `stUartRing` DMA | **已迁移** ✅ | `boart_uart_init.c` | RingBuffer + vtable |
| UART8 (SBUS) | `stUartRing` IT | **已迁移** ✅ | `boart_uart_init.c` | RingBuffer + vtable |
| USART3 (Jetson) | 旧 `Drv_Uart.c` | ❌ 未迁移 | — | 预留 |
| USART6 (GD) | 旧 `Drv_Uart.c` | ❌ 未迁移 | — | 预留 |

## 文件清单

| 文件 | 层 | 职责 |
|------|----|------|
| `uart_base.h` | Base | `stUartBase` 结构体、`stUartOps` vtable 定义 |
| `uart_base.c` | Base | 分发函数实现（含 `uart_Ring_tx_complete_isr`） |
| `uart_true.h` | Subclass | `stUartRing` 子类结构体定义（含 TX 队列字段） |
| `uart_true.c` | Subclass | IT/DMA 模式实现 + 两套 ops 表 + 非阻塞 TX 队列 |
| `ring_buffer.h` | 通用 | 环形缓冲区（RingBuffer）结构体定义 |
| `ring_buffer.c` | 通用 | RingBuffer 实现 |
| `fifo.h` | 通用 | 旧 FIFO 定义（用于未迁移的 UART） |
| `fifo.c` | 通用 | 旧 FIFO 实现 |
| `uarts.h` | Board | 声明全局 `stUartBase*` 句柄 |
| `boart_uart_init.c` | Board | 硬件绑定初始化（所有已迁移 UART） |
| `Drv_Uart.h` | (旧) | 旧驱动头文件 |
| `Drv_Uart.c` | (旧) | HAL 回调 + 数据检查 + fputc |

## 核心规则

1. **vtable 函数指针参数必须是 `stUartBase*`**，子类实现里用 `container_of` 转型。
2. **子类结构体第一个成员必须是 `stUartBase stBase`**。
3. **`ops` 是 `const` 指针**，所有 ops 表实例为 `const static`。
4. **发送缓冲区不属于 UART 驱动** — `transmit` 接收 `(data, len)` 参数，数据由调用方管理。
5. **Board 层是唯一分配数组和实例的地方**。
6. **UART 层不持有任何协议指针** — 数据消费是上层的事，UART 只搬字节。
7. **DMA TX buffer 必须在 `.dma_buf` 段（non-cacheable）**，否则需手动 `SCB_CleanDCache_by_Addr`。
8. **`osSemaphoreId_t` 直接传值**而非指针——引用 `me->pBinary_SemaphoreHandle` 直接调用 FreeRTOS API。

## 新增 UART 实例步骤

1. 在 `boart_uart_init.c` 中：
   - 分配 RingBuffer 和存储数组
   - 分配 `.dma_buf` TX buffer（DMA 端口需要）
   - 创建 `stUartRing` 实例，调用 `uartRing_init_dma()` 或 `uartRing_init_it()`
   - 暴露基类指针到 `uarts.h`
2. 在 `Drv_Uart.c` 的 `HAL_UARTEx_RxEventCallback` / `HAL_UART_TxCpltCallback` / `HAL_UART_ErrorCallback` 中添加 if 分支
3. 协议层通过 `uc_uart_read_ucbyte()` 轮询消费

## 已知问题

1. **IT ops 仍有阻塞发送** — `uart_Ring_tx_IT` 保留信号量阻塞模式（`xSemaphoreTake(portMAX_DELAY)`），后续可按需改为非阻塞队列版。
2. **中断回调仍使用 if 链** — `Drv_Uart.c` 中的回调函数仍用 `if (huart->Instance == ...)` 判断。迁移完成后可改为查表。
3. **`xTxQuene_t` 不支持 DMA 模式位中断（无传输完成中断的场景）** — 当前所有 DMA TX 都有 `HAL_UART_TxCpltCallback`。

## 已修复问题

### DMA 接收数据重复 Bug (2025-01)

**问题现象：**
- Python 发送新 MAP 命令时，STM32 会重复处理之前发送的旧数据
- 每发送一次新数据，之前所有的旧数据都会被重新处理一次
- 例如：第1次发送触发1次处理，第2次发送触发2次处理（旧+新），第3次触发3次（旧+旧+新）

**根本原因：**
`HAL_UARTEx_ReceiveToIdle_DMA` 回调中的 `Size` 参数是**从接收开始的累积字节数**，而不是自上次回调以来的增量。

原 `uart_ring_add` 代码：
```c
// 错误：每次都从 pucRxbuffer[0] 开始复制 usLen 字节
for (int i = 0; i < usLen; i++) {
    ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
}
```

当 `Size` 从 16 增长到 38 时，代码会把 `pucRxbuffer[0..37]` 全部写入环形缓冲区，包括之前已经写入过的旧数据。

**修复方案：**
记录每个 UART 实例的上次读取位置，只写入新数据部分：

```c
/* 每个 UART 实例记录上次读取位置 */
static volatile uint16_t g_last_rx_size[3] = {0};  // 最多 3 个 UART

static void uart_ring_add(stUartBase* stBase, uint16_t usLen)
{
    stUartRing *me = container_of(stBase, stUartRing, stBase);
    
    // 找到对应的 UART 索引
    int idx = -1;
    if (stBase == pstbase_ground_uart) idx = 0;
    else if (stBase == pstbase_screen_uart) idx = 1;
    else if (stBase == pstbase_usart3_uart) idx = 2;
    
    if (idx < 0) return;
    
    // 计算新数据的起始位置和长度
    uint16_t last_size = g_last_rx_size[idx];
    uint16_t new_start = last_size;
    uint16_t new_len = usLen - last_size;
    
    // 处理 DMA 缓冲区回绕的情况
    if (usLen < last_size) {
        new_start = 0;
        new_len = usLen;
    }
    
    // 只写入新数据
    for (uint16_t i = 0; i < new_len; i++) {
        uint16_t pos = new_start + i;
        if (pos >= me->usRxbuffer_size) {
            pos -= me->usRxbuffer_size;  // 回绕
        }
        ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[pos]);
    }
    
    // 更新上次读取位置
    g_last_rx_size[idx] = usLen;
}
```

**关键要点：**
- `HAL_UARTEx_ReceiveToIdle_DMA` 在循环模式下的 `Size` 是累积值（从 DMA 启动开始的总字节数），不是增量
- ~~解决方案：每次回调后重启 DMA~~ → **已修复为 `usLastRxPos` 增量追踪方案（见下方）**

**为什么 ground_uart (15字节) 没出问题：**
- ANO 协议帧小（10-20字节），飞控连续发送
- 处理及时，Size 累积量小，不会导致明显问题
- screen_uart 是离散命令，有延迟，导致 Size 累积明显

**修复方案（保持 DMA 循环模式）：**

扩展 `stUartRing` 结构体，添加 `usLastRxPos` 字段追踪 DMA 位置：
```c
typedef struct {
    // ... 其他字段 ...
    volatile uint16_t usLastRxPos;  // 上次读取的 DMA 位置
} stUartRing;
```

`uart_ring_add` 计算增量，只写入新数据：
```c
static void uart_ring_add(stUartBase* stBase, uint16_t usLen)
{
    stUartRing *me = container_of(stBase, stUartRing, stBase);
    uint16_t usLastPos = me->usLastRxPos;

    if (usLen >= usLastPos) {
        // 正常：从 usLastPos 到 usLen 是新数据
        for (uint16_t i = usLastPos; i < usLen; i++) {
            uint16_t pos = i % me->usRxbuffer_size;
            ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[pos]);
        }
    } else {
        // DMA 回绕：先写 usLastPos 到末尾，再写 0 到 usLen
        for (uint16_t i = usLastPos; i < me->usRxbuffer_size; i++) {
            ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
        }
        for (uint16_t i = 0; i < usLen; i++) {
            ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
        }
    }

    me->usLastRxPos = usLen;
}
```

**优点：**
- 保持 DMA 循环模式，无需重启
- 每个 UART 实例独立追踪位置
- 正确处理 DMA 缓冲区回绕
