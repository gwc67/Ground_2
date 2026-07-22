# User_Ctrl — 用户控制与数据标志位

## 模块职责

本目录包含两个独立的子模块，分别处理**用户操作状态**和**传感器数据更新标志**：

| 文件 | 类型 | 职责 | 调用周期 |
|------|------|------|---------|
| `user_ctrl.c/h` | 对象（不透明指针） | 解锁边沿检测 + 起飞授权 | 10ms |
| `update.c/h` | 值（全局标志位） | 传感器数据到达标志 | 事件驱动 |

## user_ctrl — 解锁状态 + 起飞授权

### 设计思路

原始 `Check_unlock_v()` 的问题：
- 全局变量 `is_unlocked_uc` / `switch_pressed_before_unlock_uc` 散落在文件顶层，外部可直接修改
- `is_unlocked_uc` 与 `Fc_state_is_unlocked_b()` 语义重复
- 手动边沿检测（`if (current != last)`）散落在业务代码中

优化后采用 **Linux 内核不透明指针模式**：

```
外部调用者                  user_ctrl.c（内部）
─────────────              ──────────────────
user_ctrl_is_takeoff_armed() ← s_dev_st.takeoff_aremd_b
                              ↑
                    user_ctrl_check_v() 每 10ms 检测边沿
                              ↑
                    Fc_state_is_unlocked_b() + Fc_state_get_SWD_em()
```

### API

```c
// 10ms 周期调用 — 检测解锁/上锁边沿
void user_ctrl_check_v(void);

// 查询是否允许起飞（解锁 + SWD 拨下）
bool user_ctrl_is_takeoff_armed(void);
```

### 起飞授权条件

```
解锁上升沿（0→1）:
  takeoff_armed = (SWD == SWITCH_LOW_em)
  // 解锁瞬间检查安全开关，锁定授权状态

上锁下降沿（1→0）:
  takeoff_armed = false
  // 上锁时清除授权
```

**关键**：`takeoff_armed` 只在解锁瞬间采样 SWD 状态。解锁后拨动 SWD 不会改变授权。这是安全设计——防止飞行中误触开关。

### 初始化

```c
static void s_user_ctrl_init(void) { ... }
DRIVER_INIT(s_user_ctrl_init);
```

通过 `DRIVER_INIT` 自动注册，无需手动调用。

## update — 传感器数据标志位

### 设计思路

多个外设（雷达、光流、相机）异步接收数据。消费者（控制任务）需要知道"有没有新数据"。`update` 模块管理 N 个独立的标志位，提供 **consume/set** 语义：

- **生产者**（ANO 协议回调）：`update_flag_set_v(field, 1)` 标记数据到达
- **消费者**（控制任务）：`update_flag_consume_uc(field)` 读取并自动清零

### Linux 内核风格：数组代替 struct + switch

**旧版问题**：每新增一个标志位，需要修改 5 处：
1. `struct update_Flag_t` 加字段
2. `enum update_flag_field_e` 加枚举值
3. `update_flag_consume_uc` 加 switch case
4. `update_flag_set_v` 加 switch case
5. `update_flag_st` 初始化器加初始化

**新版方案**（参考 Linux 内核表驱动模式）：
- 内部用 `struct update_flag_field { const char *name; uint8_t val; }` 数组
- enum 值直接作为数组下标
- 新增标志位**只需改 1 处**：在 `s_fields[]` 加一行 + enum 追加一个值
- `consume/set/dump` 函数体完全不需要修改

```c
// update.c 内部：
static struct update_flag_field s_fields[] = {
    [UPDATE_FLAG_RADAR_SPEED_em]   = { .name = "RADAR_SPEED",   .val = 0 },
    [UPDATE_FLAG_RADAR_POS_em]     = { .name = "RADAR_POS",     .val = 0 },
    // ... 新增字段追加到末尾即可
};

// consume — 无 switch，无 if-else
uint8_t update_flag_consume_uc(enum update_flag_field_e field_uc)
{
    if (!field_valid(field_uc)) return 0;
    uint8_t val = s_fields[field_uc].val;
    s_fields[field_uc].val = 0;   // 读后清零
    return val;
}
```

### 标志位列表

| 枚举值 | 含义 | 生产者 | 消费者 |
|--------|------|--------|--------|
| `UPDATE_FLAG_RADAR_SPEED_em` | 雷达速度到达 | ANO 0x07 回调 | 速度融合任务 |
| `UPDATE_FLAG_RADAR_POS_em` | 雷达位置到达 | ANO 0x34 回调 | 位置融合任务 |
| `UPDATE_FLAG_CAM_PIX_em` | 相机像素到达 | 视觉回调 | — |
| `UPDATE_FLAG_RADAR_CMD_VEL_em` | 雷达指令速度 | — | — |
| `UPDATE_FLAG_RADAR_PID_CMD_VEL_em` | 雷达 PID 指令速度 | — | — |
| `UPDATE_FLAG_CAMERA_PID_CMD_VEL_em` | 相机 PID 指令速度 | — | — |
| `UPDATE_FLAG_RADAR_QUA_em` | 雷达四元数到达 | ANO 0x04 回调 | 姿态融合任务 |
| `UPDATE_FLAG_ANO_OF1_SPEED_em` | 光流速度到达 | ANO 光流回调 | 速度融合任务 |
| `UPDATE_FLAG_ANOOF_ALT_em` | 光流高度到达 | ANO 光流回调 | 高度融合任务 |
| `UPDATE_FLAG_CAMMER_STATE_em` | 相机状态 | — | — |

### API

```c
// 消费者：读取标志位并自动清零（防止重复处理）
// 语义等同于 Linux 的 xchg() / test_and_clear_bit()
uint8_t update_flag_consume_uc(enum update_flag_field_e field_uc);

// 生产者：设置标志位
// 类似 Linux 的 set_bit() / WRITE_ONCE()
void update_flag_set_v(enum update_flag_field_e field_uc, uint8_t val_uc);

// 调试用：打印所有标志位状态
// 替代旧版 c_get_update_flag_pst() 返回 struct 指针的做法
void update_flag_dump_v(void);
```

### 新增标志位指南

只需 2 步：

```c
// 1. update.h — 在 enum 末尾追加（保持顺序与 s_fields[] 一致）
enum update_flag_field_e {
    ...
    UPDATE_FLAG_NEW_DEVICE_em,   // ← 追加到末尾
};

// 2. update.c — 在 s_fields[] 中追加一行
static struct update_flag_field s_fields[] = {
    ...
    [UPDATE_FLAG_NEW_DEVICE_em] = { .name = "NEW_DEVICE", .val = 0 },
};
```

`consume` / `set` / `dump` 函数体无需任何修改。

### 使用模式

```c
// 生产者（ANO 协议回调中）
void on_radar_speed_received(void) {
    update_flag_set_v(UPDATE_FLAG_RADAR_SPEED_em, 1);
}

// 消费者（控制任务中）
void speed_fusion_task(void) {
    if (update_flag_consume_uc(UPDATE_FLAG_RADAR_SPEED_em)) {
        // 有新数据，执行融合
        struct Radar_Speed_t speed;
        Radar_Speed_Copy(&speed);  // 获取数据快照
        // ...
    }
}
```

### 封装说明

旧版 `c_get_update_flag_pst()` 返回 `const struct update_Flag_t*`，外部可以丢弃 const 直接解引用（Keil 仅 warning），无法真正保护数据。

新版删除了该函数，替换为 `update_flag_dump_v()` 直接打印。外部代码**完全无法**拿到内部数据的可写指针，只能通过 enum 索引访问。这符合项目 CLAUDE.md 中"不提供能拿到全局数据可写指针的函数"的铁律。

## 与旧代码对比

| 旧代码 | 新架构 |
|--------|--------|
| `uint8_t is_unlocked_uc;` 全局变量 | `static struct UserCtrl_t s_dev_st;` 不透明 |
| 外部直接读 `is_unlocked_uc` | `user_ctrl_is_takeoff_armed()` 访问器 |
| `uint8_t update_flag;` 分散定义 | `s_fields[]` 数组 + 枚举下标 |
| 手动 `if (flag) { flag = 0; ... }` | `update_flag_consume_uc()` 读后清零 |
| struct + switch × 2（consume + set） | 数组下标直接访问，无 switch |
| 新增标志位改 5 处 | 新增标志位改 1 处（数组 + enum） |
| 混合边沿检测和状态变量 | 边沿检测封装在 `user_ctrl_check_v()` |

## 调用路径

```
Task_LX (1ms)
  └─ 每 10ms:
      └─ user_ctrl_check_v()          ← 检测解锁边沿
          ├─ Fc_state_is_unlocked_b()  ← 读取 IMU 解锁状态
          └─ Fc_state_get_SWD_em()     ← 读取 SWD 开关位置

ANO 协议回调 (异步):
  └─ update_flag_set_v(field, 1)       ← 标记数据到达

控制任务 (10ms/20ms):
  └─ update_flag_consume_uc(field)     ← 读取并清零
      └─ 有新数据 → 执行融合/控制
```

## 文件清单

| 文件 | 职责 |
|------|------|
| `user_ctrl.c` | 不透明类型 + 边沿检测 + 起飞授权 |
| `user_ctrl.h` | 公开 API（不暴露结构体定义） |
| `update.c` | 标志位实现（consume/set） |
| `update.h` | 标志位枚举 + API |
