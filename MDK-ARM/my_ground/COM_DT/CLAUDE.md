# COM_DT — ANO 协议引擎 vtable 架构

## 概述

ANO 协议 V7.15 通信引擎的 vtable 重构。项目中有两条使用完全相同协议格式的通路：

| 通路 | 串口 | 方向 | 原文件 |
|------|------|------|--------|
| IMU/飞控 | UART4 | STM32 ↔ 凌霄 IMU | `FcSrc/ANO_DT_LX.c` |
| 上位机 | USART1 | STM32 ↔ PC 地面站 | `My_FcSrc/COM_DT/COM_DT.c`（待迁移）|

## 铁律

**全局 Base 指针外部调用时，不能直接使用 `base->ops->xxx()`。只能通过 `ano_base.h` 中暴露的公共函数（`vano_` 前缀）间接操作。**

## 四层架构

```
App 层          Ano_Scheduler.c / freertos.c
                 ↕ stAnoBase* 全局句柄 + vano_xxx() 公共函数
Subclass 层     ano_ture.c（公共协议逻辑：stAnoDevice 子类 + stAnoOps 全局表）
                 ↕ container_of + unique 函数指针
Device 层       ano_device/*.c（通路差异化实现：Receive_Anl / Add_Send_Data / Send_Buff）
                 ↕ unique 表绑定 + init 注册
Board 层        ano_board_init.c（实例化 + 绑定 unique + DRIVER_INIT 注册）
```

## 文件职责

| 文件 | 层 | 职责 |
|------|----|------|
| `ano_base.h` | Base | `stAnoBase` + `stAnoOps` vtable 定义 + `vano_` 公共函数声明 |
| `ano_base.c` | Base | `vano_` 分发函数实现（薄 wrapper：assert + `me->ops->xxx(me, ...)`）|
| `ano_ture.h` | Subclass | `stAnoDevice` 子类 + `unique` 函数指针表定义 + `Ano_Device_init()` |
| `ano_ture.c` | Subclass | 公共协议逻辑实现：状态机、帧组装、调度、CK 重传、**数据检查（datacheck）** |
| `ano_device/ano_device_com.h` | Device | COM 通路差异化函数声明（`vCom_` 前缀）|
| `ano_device/ano_device_com.c` | Device | COM 通路实现：初始化、帧解析、数据填充、发送 |
| `ano_device/ano_device_lx_dt.h` | Device | LX 通路差异化函数声明（`vLx_` 前缀）|
| `ano_device/ano_device_lx_dt.c` | Device | LX 通路实现：初始化、帧解析（含四元数→欧拉角转换）、数据填充、发送 |
| `ano_board_init.c` | Board | 实例化 stAnoDevice_Com + stAnoDevice_Lx + 绑定 unique + DRIVER_INIT 注册 |
| `ano.h` | Board | `extern stAnoBase* pstAnobase_Com` + `extern stAnoBase* pstAnobase_Lx` + `ano_board_init()` 声明 |

## Base 层 — `ano_base.h`

```c
typedef struct {
    void (*data_receive_prepare)(stAnoBase *me, uint8_t ucdata);
    void (*cmd_send)(stAnoBase *me, u8 dest_addr, _cmd_st *cmd);
    void (*ck_back)(stAnoBase *me, u8 dest_addr, _ck_st *ck);
    void (*par_back)(stAnoBase *me, u8 dest_addr, _par_st *par);
    void (*sendID_set)(stAnoBase *me, u8 ucFrame_num, u16 usFreq);
    void (*check_to_send)(stAnoBase *me, u8 ucFrame_num);
    void (*WTS_set)(stAnoBase *me, u8 ucFrame_num, uint8_t ucState);
    void (*wait_ck_clear)(stAnoBase *me);
    void (*ck_get)(stAnoBase *me, _ck_st *pstck);
    void (*cmd_get)(stAnoBase *me, u8 dest_addr, _cmd_st *pstcmd);
    void (*par_get)(stAnoBase *me, u8 dest_addr, _par_st *pstpar);
    _dt_st* (*pstdtdevice_get)(stAnoBase *me);
    void (*ano_ck_back_check)(stAnoBase *me);
    void (*send_string)(stAnoBase *me, s32 lValue, char *pcstr);
    void (*ano_check_data)(stAnoBase *me, stUartBase* pstbase_uart);  // ← datacheck 移至此处
} stAnoOps;

struct stAnoBase { stAnoOps *ops; };
```

外部通过 `ano_base.h` 的 `vano_` 分发函数调用：

```c
vano_data_receive_prepare(me, ucdata);
vano_cmd_send(me, addr, &cmd);
vano_ck_back(me, addr, &ck);
vano_par_back(me, addr, &par);
vano_ck_get(me, &ck);
vano_cmd_get(me, addr, &cmd);
vano_par_get(me, addr, &par);
vano_sendID_set(me, frame_num, freq);
vano_check_to_send(me, frame_num);
vano_WTS_set(me, frame_num, state);
vano_wait_ck_clear(me);
vano_ck_back_check(me);
pstano_dtdevice_get(me);
vano_send_string(me, lValue, str);
vano_check_data(me, pstbase_uart);           // ← 新增：轮询 UART RingBuffer 消费数据
```

## Subclass 层

### stAnoDevice 子类

```c
typedef struct {
    stAnoBase stBase;           // 基类（必须第一个）
    _dt_st* pstDeviceDt;        // 指向外部 _dt_st 实例
    u8* pucTxBuffer;            // 发送缓冲区
    u8* pucRxBuffer;            // 接收缓冲区
    unique stUser;              // 特殊化函数指针表
    uint8_t ucRxstate;          // 接收状态机（独立）
    uint8_t ucData_Len;         // 帧数据长度
    uint8_t ucData_Cnt;         // 帧数据计数
} stAnoDevice;
```

### unique 差异化接口

| 函数指针 | 职责 | 对应 Device 层函数 |
|----------|------|-------------------|
| `Ano_Receive_Anl` | 收到整帧后的 CMD 分发 | `vCom_DT_Data_Receive_Anl_Ano` / `vLx_DT_Data_Receive_Anl_Ano` |
| `Ano_Add_Send_Data` | 发送时填充帧载荷 | `vCom_Add_Send_Data_Ano` / `vLx_Add_Send_Data_Ano` |
| `Send_Buff` | 物理层发送 | `vCom_TxBuffer_Ano` / `vLx_TxBuffer_Ano` → `uart_transmit()`（DMA 端口非阻塞，IT 端口阻塞） |

### ano_ture.c 内部函数（全部 static）

| 函数 | 绑定的 ops 成员 | 职责 |
|------|----------------|------|
| `vData_receive_prepare_Ano` | `data_receive_prepare` | 7 级状态机收帧 |
| `vData_Check_Ano` | `ano_check_data` | **轮询 UART RingBuffer，逐字节喂给状态机** |
| `vCmd_send_Ano` | `cmd_send` | 设置 cmd_send + 触发 0xe0 发送 |
| `vCk_back_Ano` | `ck_back` | 填写校验回执 + 触发 0x00 发送 |
| `vPar_back_Ano` | `par_back` | 填写参数 + 触发 0xe2 发送 |
| `vPar_get_Ano` | `par_get` | 读取 par_data |
| `vCmd_get_Ano` | `cmd_get` | 读取 cmd_send |
| `vFrame_Send_Ano` | — | 内部帧组装 + 填充数据 + 校验码 + 发送 |
| `vCheck_to_send_Ano` | `check_to_send` | 定时发送调度 + WTS 触发 |
| `vFrame_num_set_Ano` | `sendID_set` | 设置帧频率 |
| `vWTS_Set_Ano` | `WTS_set` | 设置 WTS 标志 |
| `Wait_ck_clear_Ano` | `wait_ck_clear` | 清 wait_ck + 触发队列中待发 CMD |
| `vCk_Get_Ano` | `ck_get` | 读取 ck_send |
| `pstDtDevice_Get_Ano` | `pstdtdevice_get` | 返回 _dt_st* |
| `vCK_Back_Check_Ano` | `ano_ck_back_check` | CK 超时重传（最长 5 次） |
| `vSend_String_Ano` | `send_string` | 发送调试字符串帧 (0xA1) |

### 接收状态机流程

```
state 0: 收到 0xAA → state 1
state 1: 收到 0xFF → state 2，否则复位
state 2: 收到 CMD  → state 3（存 pucRxBuffer[2]）
state 3: 收到 LEN  → state 4（存 pucRxBuffer[3]），记录数据长度
state 4: 收 DATA（len 字节）→ state 5
state 5: 收到 SUM1  → state 6
state 6: 收到 SUM2  → 复位，回调 Ano_Receive_Anl（整帧在 pucRxBuffer）
```

### vData_Check_Ano — 数据消费的核心变动

```c
static void vData_Check_Ano(stAnoBase *stBase, stUartBase* pstbase_uart)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    uint8_t data_temp;
    while (uc_uart_read_ucbyte(pstbase_uart, &data_temp) == 0)  // 从 UART RingBuffer 读
    {
        vData_receive_prepare_Ano(&me->stBase, data_temp);       // 喂给状态机
    }
}
```

对比旧的架构：`uart_datacheck` 在子类 `stUartTrue` 内部硬编码调用 `GetOneByte`。现在 ANO 层通过 `uc_uart_read_ucbyte()` 主动读取，**UART 层不再持有协议指针**。

## Device 层 — `ano_device/*.c`

Device 层按通路拆分到子目录，每条通路通过 `unique` 表绑定到 Subclass 层：

```c
// ano_device/ano_device_com.c
static unique stUser_Com = {
    .Ano_Add_Send_Data = vCom_Add_Send_Data_Ano,
    .Ano_Receive_Anl   = vCom_DT_Data_Receive_Anl_Ano,
    .Send_Buff         = vCom_TxBuffer_Ano,
};

// ano_device/ano_device_lx_dt.c
static unique stUser_Lx = {
    .Ano_Add_Send_Data = vLx_Add_Send_Data_Ano,
    .Ano_Receive_Anl   = vLx_DT_Data_Receive_Anl_Ano,
    .Send_Buff         = vLx_TxBuffer_Ano,
};
```

### CMD 帧处理（COM 通路）

| CMD | 行为 |
|-----|------|
| `0xE0` | 命令帧：case 0x01 触发 GX_Sensor 发送；case 0x02 触发 0xe0 命令发送。始终返回 CK 校验帧 |
| `0xE1` | 读参数帧：读取 par_id 后返回 par_back（含 par_val=0）|
| `0x00` | 校验确认帧：比对 ck_send 校验和，匹配则清 wait_ck |

### CMD 帧处理（LX 通路）

| CMD | 行为 |
|-----|------|
| `0x20` | 解析 8 路 PWM 电机值 → `pwm_to_esc` |
| `0x0f` | 解析 4 路 LED 亮度 → `led_set_brightness` |
| `0x06` | 解析飞控状态 → `fc_sta` |
| `0x07` | 解析速度 → `fc_vel` |
| `0x04` | 解析四元数 → 计算欧拉角 → `fc_att` + `fc_att_qua` |
| `0xE0` | 命令帧：返回 CK 校验帧 |
| `0x00` | 校验确认帧：比对 ck_send |
| `0xE1` | 读参数帧：返回 par_back |
| `0xE2` | 参数写入：返回 CK 校验帧 |

### Add_Send_Data 数据帧（COM 通路）

| 帧号 | 填充内容 |
|------|---------|
| `0x00` | ck_send.ID/SC/AC（校验回执）|
| `0x01` | 测试传感器数据（递增 s16 值）|
| `0x03` | fc_att 欧拉角 |
| `0x04` | fc_att_qua 四元数 |
| `0x06` | fc_sta 状态（模式/解锁/指令）|
| `0x07` | fc_vel 速度 |
| `0x0f` | LED 亮度 |
| `0x20` | PWM_ESC 8 路电机值 |
| `0xe0` | cmd_send.CID + CMD[0..9] |
| `0xe2` | par_data.par_id + par_val |

### Add_Send_Data 数据帧（LX 通路）

| 帧号 | 填充内容 |
|------|---------|
| `0x00` | ck_send.ID/SC/AC（校验回执）|
| `0x0d` | fc_bat 电池电压 |
| `0x30` | ext_sens.fc_gps GPS 数据（23 字节）|
| `0x33` | ext_sens.gen_vel 通用速度 |
| `0x34` | ext_sens.gen_dis 通用距离/高度 |
| `0x40` | rc_in.rc_ch 遥控通道（10 通道值）|
| `0x41` | rt_tar 实时目标值 |
| `0xe0` | cmd_send.CID + CMD[0..9] |
| `0xe2` | par_data.par_id + par_val |

### 数据发送任务

```c
// COM 通路
void vCom_Data_Exchange_Task_Ano(void)
{
    vano_ck_back_check(pstAnobase_Com);
    vano_check_to_send(pstAnobase_Com, 0x00);
    vano_check_to_send(pstAnobase_Com, CMD_Zhen);
    vano_check_to_send(pstAnobase_Com, GX_Sensor);
    vano_check_to_send(pstAnobase_Com, OL_Angle);
    vano_check_to_send(pstAnobase_Com, QUA_BYTE);
    vano_check_to_send(pstAnobase_Com, RUN_MODE);
    vano_check_to_send(pstAnobase_Com, RUN_SPEED);
    vano_check_to_send(pstAnobase_Com, RGB_LED);
    vano_check_to_send(pstAnobase_Com, PWM_ESC);
    vano_check_to_send(pstAnobase_Com, PAR_WRITE);
}

// LX 通路
void vLx_Data_Exchange_Task_Ano(void)
{
    vano_ck_back_check(pstAnobase_Lx);
    vano_check_to_send(pstAnobase_Lx, LX_GPS);
    vano_check_to_send(pstAnobase_Lx, LX_GEN_VEL);
    vano_check_to_send(pstAnobase_Lx, LX_GEN_DIS);
    vano_check_to_send(pstAnobase_Lx, LX_RC_CH);
    vano_check_to_send(pstAnobase_Lx, LX_RT_TAR);
    vano_check_to_send(pstAnobase_Lx, LX_CMD);
    vano_check_to_send(pstAnobase_Lx, LX_PAR);
    vano_check_to_send(pstAnobase_Lx, LX_BAT);
}
```

## 与 UART vtable 的关系

```
ANO Proto vtable      ← 管"帧语义" + 数据消费
    ↓ vano_check_data() 内部 → uc_uart_read_ucbyte()
    ↓ unique.Send_Buff()    → uart_transmit()
        ↑
UART vtable           ← 管"字节搬运"（DMA 端口非阻塞队列发送）

重要: UART 层不持有任何协议指针。
      ANO 层通过 UART 的公共读接口 (uc_uart_read_ucbyte) 主动轮询。
      uart_transmit() 对 DMA 端口是非阻塞的——ANO 帧入队后立即返回，不等待发送完成。
```

## 全局 ops 表

`ano_ture.c` 中定义的 `ano_operate` 是两路设备**共享的**，`unique` 表才是差异化的关键。

## 设计要点

1. **Device 层不持有内部指针** — 所有对 `_dt_st` 内部字段的读写通过 `vano_` API 完成
2. **状态机实例化** — `ucRxstate`/`ucData_Len`/`ucData_Cnt` 在 `stAnoDevice` 中，每个设备独立
3. **`ck_get` 读 `ck_send`** — 读的是"我要发送的校验回执"，而非"我收到的校验记录"
4. **CK 重传机制** — `wait_ck=1` 时，每 50 滴答重发 0xe0，最多 5 次后清 wait_ck
5. **Frame_Send 中 ck_back 字段** — 记录已发帧的校验和，供收到 0x00 时比对
6. **`datacheck` 在 ANO 层而非 UART 层** — `vData_Check_Ano()` 通过 `uc_uart_read_ucbyte()` 从 UART RingBuffer 读取，逐字节喂给 ANO 状态机。UART 层不再知道任何协议细节

## 迁移状态

| 组件 | 状态 |
|------|------|
| Base 分发层 | ✅ 完成 |
| Subclass 公共逻辑（含 datacheck） | ✅ 完成 |
| COM 通路 Device 层 | ✅ 完成 |
| LX 通路 Device 层（含四元数→欧拉角） | ✅ 完成 |
| Board 初始化（双路实例化） | ✅ 完成 |
| 替换旧 `COM_DT.c` 调用 | ❌ 未开始 |
| 替换旧 `ANO_DT_LX.c` 调用 | ❌ 未开始 |

## 引用

- `ANO_LX.h` — `_dt_st`、`_cmd_st`、`_ck_st`、`_par_st` 定义
- `ANO_LX.h` — `fc_att`、`fc_att_qua`、`fc_vel`、`fc_sta` 全局数据
- `Drv_Uart/uart_base.h` — `uart_transmit`、`uc_uart_read_ucbyte` 等 UART 分发函数
- `Drv_Uart/uarts.h` — `pstbase_com_uart`、`pstbase_lx_uart` UART 句柄
