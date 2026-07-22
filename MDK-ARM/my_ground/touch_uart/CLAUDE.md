# touch_uart — 屏幕串口命令解析层

## 概述

解析 Python 上位机通过 USART2（屏幕串口）发送的 ASCII 文本命令，将用户在地面站界面上的操作（设置禁飞区、请求巡逻路径、请求返航路径）转化为对 `Mission` 层的函数调用。

**职责边界：** 只做协议解析 + 调度，不存储任何任务状态。所有业务逻辑下沉到 `Mission/mission_planner.c`。

## 在系统中的位置

```
Python 上位机 (Ground Station GUI)
    │  USART2 (ASCII, 换行分隔)
    ▼
touch_uart  ←── 本模块：行组装 + 命令分派
    │
    ▼
Mission/mission_planner
    │  mission_set_no_fly_zones()
    │  mission_python_request_patrol()
    │  mission_python_request_return()
    ▼
Route_Planning → 飞控
```

**调用链：** `DrvUartDataCheck()`（1 ms 轮询） → `screen_uart_check()` → 逐字节读取 → 行组装 → `dispatch_line()` → Mission 层函数。

## 支持的命令

| 命令格式 | 说明 | 处理函数 |
|---------|------|---------|
| `MAP:x1,y1,x2,y2,x3,y3\n` | 更新禁飞区坐标（最多 3 个网格点，1-indexed） | `parse_map_command()` |
| `REQUEST_PATH\n` | 请求规划巡逻路径并发送给上位机 | → `mission_python_request_patrol()` |
| `REQUEST_RETURN\n` | 请求规划返航路径并发送给上位机 | → `mission_python_request_return()` |

未知命令静默忽略，不回复。

## 协议细节

### MAP 命令

```
MAP:3,4,5,6,7,2\n
     └── 6 个整数，逗号分隔
         ├── coords[0..1] → zones[0] = (3, 4)
         ├── coords[2..3] → zones[1] = (5, 6)
         └── coords[4..5] → zones[2] = (7, 2)
```

- `MAP:` 前缀后紧跟数据，无空格
- 不足 6 个坐标时，按实际数量解析（至少 2 个 = 1 个禁飞区才执行）
- 多余坐标被忽略（`for` 循环硬编码 6 次）
- 调用 `mission_set_no_fly_zones()` 时**覆盖**当前所有禁飞区，非增量

### REQUEST_PATH / REQUEST_RETURN

无参数，纯触发命令。路径规划结果由 Mission 层通过串口发回上位机（不在本模块处理）。

## 行组装机制

```c
static char s_line_buf[128];   // 行缓冲区（static 避免栈溢出）
static uint8_t s_line_pos = 0; // 当前写入位置
```

- 逐字节从 `uc_uart_read_ucbyte(pstbase_screen_uart, &byte)` 读取
- `\n` 或 `\r` 触发 `dispatch_line()`，然后重置 `s_line_pos = 0`
- 连续的 `\n` / `\r` 被跳过（`s_line_pos == 0` 时不触发）
- **溢出处理：** 超过 127 字节未遇换行 → 丢弃当前行，重置 `s_line_pos = 0`
- **无校验和 / CRC** — 依赖上位机保证格式正确

## 核心规则

1. **本模块不持有任何状态** — 禁飞区、路径缓存等全部在 Mission 层。`touch_uart` 只做解析和转发。
2. **static 缓冲区** — `s_line_buf` 是文件内 static，避免 128 字节占栈（嵌入式栈空间有限）。
3. **不可重入** — 只有 `screen_uart_check()` 一个入口，只在 1 ms 轮询任务中调用，无并发问题。
4. **阻塞读取安全** — `while (uc_uart_read_ucbyte(...) == 0)` 循环只读取 RingBuffer 中已有数据，不会阻塞等待新字节。
5. **坐标约定** — MAP 命令中的坐标是 1-indexed 网格坐标（与 `struct Point` 在 Mission 层的约定一致），直接透传不做转换。

## 依赖

| 头文件 | 用途 |
|-------|------|
| `uart_base.h` | `uc_uart_read_ucbyte()` 字节读取接口 |
| `uarts.h` | `pstbase_screen_uart` 全局句柄（USART2） |
| `Mission/mission_planner.h` | 业务函数声明 |
| `Common/types.h` | `struct Point` 定义 |

## 文件清单

| 文件 | 职责 |
|------|------|
| `touch_uart.h` | 公共接口声明（仅 `screen_uart_check`） |
| `touch_uart.c` | 行组装 + 命令解析 + 分派 |

## 扩展新命令

1. 在 `dispatch_line()` 中增加 `else if` 分支
2. 如需参数解析，新增 `static` 解析函数（参照 `parse_map_command` 模式）
3. 调用 Mission 层对应函数，不要在 `touch_uart.c` 中添加状态存储
4. 确认 `LINE_BUF_SIZE`（128）足够容纳最长命令
