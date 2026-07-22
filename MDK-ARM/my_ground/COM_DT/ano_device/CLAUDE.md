# ano_device — ANO 协议设备差异化层

## 概述

按通信通路拆分的差异化实现。每个通路有自己的 `unique` 表，绑定到 `ano_ture.c` 的公共协议逻辑。

## 通路

| 通路 | 文件 | 串口 | 连接设备 |
|------|------|------|---------|
| COM | `ano_device_com.c/h` | USART1 | PC 地面站（上位机） |
| LX | `ano_device_lx_dt.c/h` | UART4 | 凌霄 IMU/飞控 |

## unique 接口表

```c
static unique stUser_Com = {
    .Ano_Add_Send_Data = vCom_Add_Send_Data_Ano,  // 填充发送帧数据
    .Ano_Receive_Anl   = vCom_DT_Data_Receive_Anl_Ano,  // 收到整帧后分发
    .Send_Buff         = vCom_TxBuffer_Ano,        // 物理层发送
};
```

```c
static unique stUser_Lx = {
    .Ano_Add_Send_Data = vLx_Add_Send_Data_Ano,
    .Ano_Receive_Anl   = vLx_DT_Data_Receive_Anl_Ano,
    .Send_Buff         = vLx_TxBuffer_Ano,
};
```

`Send_Buff` 调用 `uart_transmit()`，经过 vtable 分发到端口的具体实现（DMA 非阻塞队列）。

## 数据发送帧

### COM 通路

| 帧号 | 数据内容 |
|------|---------|
| `0x00` | ck_send 校验回执 |
| `0x01` | 测试传感器数据 |
| `0x03` | 欧拉角 |
| `0x04` | 四元数 |
| `0x06` | 飞控状态 |
| `0x07` | 速度 |
| `0x0f` | LED 亮度 |
| `0x20` | PWM 电机值 |
| `0xe0` | 命令帧 |
| `0xe2` | 参数帧 |

### LX 通路

| 帧号 | 数据内容 |
|------|---------|
| `0x00` | ck_send 校验回执 |
| `0x0d` | 电池电压 |
| `0x30` | GPS 数据 |
| `0x33` | 通用速度（光流） |
| `0x34` | 通用距离/高度 |
| `0x40` | RC 遥控通道 |
| `0x41` | 实时目标值 |
| `0xe0` | 命令帧 |
| `0xe2` | 参数帧 |

## 接收 CMD 分发

### COM 通路

| CMD | 行为 |
|-----|------|
| `0xE0` | 命令帧：触发发送 + CK 回执 |
| `0xE1` | 读参数帧：返回 par_back |
| `0x00` | 校验确认帧：比对 ck_send |

### LX 通路

| CMD | 行为 |
|-----|------|
| `0x20` | 解析 8 路 PWM → `pwm_to_esc` |
| `0x0f` | 解析 LED 亮度 |
| `0x06` | 解析飞控状态 → `fc_sta` |
| `0x07` | 解析速度 → `fc_vel` |
| `0x04` | 解析四元数 → 欧拉角 → `fc_att` |
| `0xE0` | 命令帧：返回 CK 回执 |
| `0xE1` | 读参数帧：返回 par_back |
| `0xE2` | 参数写入：返回 CK 回执 |
| `0x00` | 校验确认帧：比对 ck_send |

## 引用

- `ano_ture.c` — 公共协议逻辑（帧组装、校验、调度）
- `Drv_Uart/uart_base.h` — `uart_transmit` 物理发送
- `Drv_Uart/uarts.h` — `pstbase_com_uart` / `pstbase_lx_uart`

## 航点上传协议（地面站 → 飞控）

### 帧定义

| 帧 | 类型 | 载荷 | ACK |
|---|---|---|---|
| `0xE0/0x01` | CMD 帧 | `{sub_cmd=0x01}` | ✅ 等 ACK |
| `0x16` | 数据帧 (GS_FRAME_WAYPOINT) | `{seq, total, Point_t[]}` | ❌ fire-and-forget |

### 0x16 载荷格式

```
字节偏移  内容
[0]       seq    — 帧序号（单帧时为 0）
[1]       total  — 总帧数（单帧时为 1）
[2..5]    Point_t[0]: x(16) + y(16)
[6..9]    Point_t[1]: x(16) + y(16)
...
```

单帧最多 63 个航点（255 - 2 = 253, 253/4 = 63）。

### 发送流程

```
ground_send_patrol_waypoints_v(pts, cnt)
  │
  ├─ 缓存航点到 s_waypoint_tx_buf[]
  ├─ 设 s_waypoint_pending = 1
  └─ ground_send_waypoint_clear_v()  ──→  0xE0/0x01 CMD 帧，wait_ck = 1
                                             │
     飞控回 ACK ───────────────────────────────┘
       │
       └─ handle_ack_frame()
            ├─ vano_wait_ck_clear()
            └─ if (s_waypoint_pending)
                 ├─ s_waypoint_pending = 0
                 ├─ vano_WTS_set(0x16, 1)
                 └─ vano_check_to_send(0x16)  ──→  立即发送 0x16 数据帧
```

### 设计决策

- **clear 用 0xE0 CMD 帧**：需要 ACK 确认飞控已清空 FIFO
- **航点数据用 0x16 数据帧**：不受 CMD[10] 长度限制，单帧装下 63 点
- **不等 ACK 直接发**：地面站到飞控 USART 直连，物理层可靠
- **巡逻/返航共用 0x16**：飞控通过状态机上下文区分（不并发上传）
