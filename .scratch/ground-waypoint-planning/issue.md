---
subject: 地面站航点规划系统
labels: ready-for-agent
---

## Problem Statement

当前系统的航点规划逻辑运行在飞控（FC）本地：飞控接收地面站下发的 3 个禁飞区网格点（ANO 0xE0/0x10），在本地执行 `Route_Planning.c` 的 9×7 网格路径规划算法，生成去程 + 返航两条航线，填入两个独立的航点 FIFO，由 `fly_task.c` 状态机逐点执行。

这导致：
1. **地面站对任务无控制力** — 地面站无法动态调整航线（如发现新障碍物、动物位置变化）
2. **FC 计算资源浪费** — 飞控 MCU 承担不必要的规划计算
3. **协议不对称** — 地面站只下发禁飞区，不参与航点生成，无法实现动态任务管理
4. **FIFO 语义重复** — FC 侧 `Way_Point_Fifo` 和 `ReturnHome_WayPoint_fifo` 结构和操作完全相同，仅状态机取用的时机不同

目标：将航点规划从飞控迁移到地面站，建立双向航点通路，实现动态任务模型。

## Solution

将系统拆分为"地面站规划 + 飞控执行"的分层架构：

- **地面站**：维护全局 9×7 网格地图，接收 FC 的动物汇报，运行路径规划算法，生成世界坐标系航点，通过 ANO 协议下发
- **飞控**：合并两个 FIFO 为单一航点 FIFO，按相位执行航点（巡逻 → 悬停等待 → 返航 → 降落），同时保留本地动物追踪能力，周期性向地面站汇报
- **动物追踪（混合模式）**：飞控本地计算动物世界坐标（板载摄像头实时性要求），同时周期性汇报给地面站用于全局地图更新

任务流采用动态模型：FC 进入巡逻相位后，地面站收到事件通知，规划并下发巡逻航点；FC 执行完毕后悬停等待（30s 超时自动返航），地面站再下发返航航点。

## User Stories

1. As a 任务操作员, I want 地面站能动态下发巡逻航线给飞控, so that 我可以根据实时态势调整任务
2. As a 任务操作员, I want 地面站能动态下发返航航线给飞控, so that 返航路径可以根据当前位置优化
3. As a 飞控系统, I want 通过单一航点 FIFO 接收所有航线（巡逻/返航）, so that 代码更简洁、无 FIFO 语义重复
4. As a 飞控系统, I want 在航点 FIFO 为空时悬停等待新航点, so that 地面站有时间规划和下发后续航线
5. As a 飞控系统, I want 悬停超过 30 秒未收到新航点时自动直线返航, so that 通信中断时仍能安全返回
6. As a 飞控系统, I want 周期性上报当前相位和位置（1Hz + 事件驱动）, so that 地面站能实时掌握任务状态
7. As a 地面站, I want 收到 FC 的相位切换事件（PATROL_ENTERED, PATROL_DONE, RETURN_DONE）, so that 我知道何时该发巡逻/返航航点
8. As a 地面站, I want 收到 FC 的动物汇报（每 2s 周期）, so that 我能维护全局动物地图用于规划
9. As a 地面站, I want 将网格坐标规划结果转换为世界坐标后下发, so that FC 可以直接执行、无需知道网格坐标系
10. As a 地面站, I want 通过 ANO 0xE0/0x01 清空 FC 航点 FIFO, so that 可以重置任务
11. As a 地面站, I want 通过 ANO 0xE0/0x02 批量上传航点（带 seq/total 分片）, so that 大批量航线可以可靠传输
12. As a 飞控系统, I want 对 0xE0/0x02 航点帧按 seq 顺序接收，seq=0 时清空 FIFO, so that 分片航点可以正确重组
13. As a 飞控系统, I want 保留本地 animal_tracker 和实时追踪能力, so that 到达航点格子后仍能逐个追踪动物
14. As a 飞控系统, I want 保留动物汇报通路（已定义但未调度的 animal_tracker_check_report_v）, so that 地面站能获得全局视图
15. As a 飞控系统, I want 继续接收 0xE0/0x10 禁飞区数据, so that 兼容现有地面站输入（虽然规划已由地面站做，禁飞区信息仍可作参考）
16. As a 任务操作员, I want 地面站能根据动物汇报动态重规划航线, so that 无人机优先巡查动物密集的格子
17. As a 系统设计者, I want 航点协议支持分片（seq/total）, so that 未来航线规模扩大时不受单帧长度限制
18. As a 飞控系统, I want 在 SWD HIGH（任务暂停）期间保持现有行为, so that 不影响手动操控

## Implementation Decisions

### 架构分层

- **地面站三层**：Route_Planning（算法）→ 任务状态机（触发时机）→ ANO 协议层（发送）
- **飞控侧**：fly_task 状态机不变（PATROL → RETURN → DESCEND → LAND），仅合并 FIFO、增加悬停等待 + 超时

### ANO 协议扩展

**Ground → FC（CMD 帧 0xE0）**：

- 子命令 0x01：WayPointClear — payload 为空，清空 FC 航点 FIFO
- 子命令 0x02：AddWayPoint — payload = `{seq:u8, total:u8, Point_t[]}`
  - `seq` 当前帧序号（0-based），`total` 总帧数
  - `seq=0` 时 FC 自动清空 FIFO 再追加
  - 每 Point_t = `{s16 x, s16 y}` = 4 字节，单帧最多 ~60 个航点
  - `seq == total-1` 表示该批航点结束

**FC → Ground（数据帧）**：

- 0x14 Animal_Report：已定义（`{row:u8, col:u8, count_per_type[6]:u8[6]}`），需调度 `animal_tracker_check_report_v`（2s 周期）
- 0x15 FC_Status：新增，payload = `{phase:u8, pos_x:s16, pos_y:s16}`
  - 1Hz 周期上报位置（位置字段持续更新）
  - 相位切换时立即上报（事件驱动）
  - phase 枚举：IDLE=0, TAKEOFF=1, ALT_HOLD=2, PATROL=3, RETURN=4, DESCEND=5, LAND=6
  - 关键事件相位：PATROL_ENTERED（进入巡逻，地面站可发航点）、PATROL_DONE（巡逻 FIFO 空，地面站应发返航航点）、RETURN_DONE

### 坐标系

- 地面站规划输出：网格坐标 `Point{u8 x, y}`（1-indexed，AB 系）
- 坐标转换（地面站侧）：`world_x = (grid_y - 1) * 50`, `world_y = (9 - grid_x) * 50`
- 飞控接收：`Point_t{s16 x, y}` 世界坐标（cm）

### FC 侧 FIFO 合并

- 删除 `ReturnHome_WayPoint_fifo`，仅保留单一 `WayPoint_Fifo`（容量 126 个 Point_t）
- `Add_ReturnHomeWayPoint` → 改为 `Add_WayPoint`，FIFO 相同
- `fly_task.c` RETURN 阶段：从同一 `WayPoint_Fifo` 取点
- FIFO 空时进入悬停等待态，30s 超时 → 直线返航（fallback）

### 任务时序

```
T0: FC SWD LOW → TAKEOFF_DELAY(2s) → ALT_HOLD → PATROL
T1: FC 上报 FC_Status{phase=PATROL_ENTERED}
T2: Ground 收到 → route_planning 生成巡逻航线 → 0xE0/0x02(seq/total) 下发
T3: FC 收齐航点 → 逐点执行（到格子后触发本地动物追踪）
T4: FC FIFO 空 → 上报 FC_Status{phase=PATROL_DONE} → 悬停等待
T5: Ground 收到 PATROL_DONE → 规划返航航线 → 0xE0/0x02 下发
T6: FC 执行返航 → 上报 RETURN_DONE → DESCEND → LAND
```

### 地面站模块结构

- 新增 `my_ground/Route_Planning/route_planning.c/h`（从 FC 移植）
- 任务触发逻辑放在 `ano_device_ground.c`（监听 0x15 FC_Status 的相位事件）
- 全局动物地图：`ano_device_ground.c` 维护 9×7 地图，由 0x14 Animal_Report 更新

### 飞控侧改动

- `point_board_init.c`：合并两个 FIFO 为一个
- `points.h`：删除 `ReturnHome_WayPoint_Base_pst`，统一为 `WayPoint_Base_pst`
- `fly_task.c`：
  - `s_phase_return_home_v`：从 `WayPoint_Base_pst` 取点（删除对 `ReturnHome_WayPoint_Base_pst` 的引用）
  - 增加悬停等待态：FIFO 空 → `has_target_b=false`，记录进入悬停时间戳，超时 30s → 直接计算 Home 点
  - 上报 FC_Status（0x15）：1Hz 周期 + 相位切换事件
- `Ano_Scheduler.c`：调度 `animal_tracker_check_report_v`（2s 周期）
- `GroundStation_DT.c`：新增 0xE0/0x01（Clear）和 0xE0/0x02（AddWayPoint）子命令处理

### 地面站侧新增

- `my_ground/Route_Planning/route_planning.c/h`：
  - 从 FC 移植 `plan_path()`、`compress_path()`、`plan_return_path()` 等核心算法
  - 新增 `grid_to_world()` 坐标转换函数
  - 接口：`plan_mission(No_Fly_Zone[3], start_pos) → {patrol_pts[], patrol_cnt, return_pts[], return_cnt}`
- `ano_device_ground.c`：
  - 新增全局动物地图 `s_animal_map[9][7]`
  - 在 `handle_cmd_frame` 中新增 0x15 帧接收处理，解析 phase 字段
  - 新增任务状态机：监听 PATROL_ENTERED → 调 `plan_mission()` → 打包 0xE0/0x02 发送
  - 监听 PATROL_DONE → 规划返航 → 发送

## Testing Decisions

### 测试原则

- 只测外部行为，不测实现细节
- 优先使用现有 seam，避免新增测试基础设施
- 最高 seam 优先：协议层集成测试 > 模块单元测试

### 测试 seam（按优先级）

1. **协议集成 seam（最高）**：
   - Ground 侧：构造一个假的 FC_Status{phase=PATROL_ENTERED} 帧喂给 `vGround_DT_Data_Receive_Anl_Ano`，断言后续调用 `uart_transmit` 发出的数据包含正确的 0xE0/0x02 帧（seq/total/航点数据）
   - FC 侧：构造假的 0xE0/0x02 帧喂给 `GroundStation_Data_Receive_Anl`，断言 `WayPoint_Fifo` 中有正确的 `Point_t` 数据
   - 这是最高 seam，一个测试覆盖：Ground 规划 → 坐标转换 → 打包 → FC 接收 → 入队

2. **算法 seam**：
   - 对 `route_planning.c` 的 `plan_path()`、`compress_path()`、`grid_to_world()` 做单元测试
   - 输入：已知障碍物布局 → 输出：期望航点序列（可预先从 FC 侧运行得到参考值）

3. **状态机 seam**：
   - 模拟 FC 相位序列（PATROL_ENTERED → PATROL_DONE → RETURN_DONE），断言 Ground 在正确时机发送航点

4. **坐标转换 seam**：
   - 纯函数测试：`grid_to_world(1,1) == {0, 400}`、`grid_to_world(9,7) == {300, 0}`

### 既有 prior art

- `ano_device_ground.c` 的 `handle_cmd_frame` 已有 switch-case 框架，新增 case 可直接通过现有接收链路测试
- FC 侧 `GroundStation_DT.c` 的 0xE0/0x10 已有类似实现可参考

## Out of Scope

- **地面站 UI 显示**：本次只实现协议通路和规划算法，不包含地面站 UI 渲染航线
- **动态重规划（任务中）**：当前版本仅在 PATROL_ENTERED 和 PATROL_DONE 两个时机规划，不支持任务中途重新规划巡逻航线（可作为后续迭代）
- **禁飞区来源**：禁飞区仍由地面站预设（通过 0xE0/0x10 下发给 FC 兼容），不从 FC 反向同步
- **动物追踪精度优化**：`animal_tracker_feed_cam_pix_v` 的世界坐标计算公式（+2.5/+5 偏移）保留现状，不在本次调整
- **航点 ACK 机制**：ANO 协议本身有 SUM1/SUM2 校验，本次不增加应用层 ACK；若后续发现丢帧问题再补
- **jesnano CID 0x03 对接**：fly_task.c TODO 中提到的 `trace_points` 与 jesnano 数据格式对接，留作后续

## Further Notes

- **FC 代码修改**：本 PRD 要求修改 FC 侧代码（合并 FIFO、增加 FC_Status 上报、调度 animal_tracker_check_report_v）。这些改动在 FC 仓库 `D:\Downloads\stm32project\mydrone2\My_FcSrc` 中进行，与地面站仓库 `D:\Downloads\stm32project\Ground` 独立
- **协议兼容**：0xE0/0x10 禁飞区帧保留，FC 仍可接收（虽然不再用于本地规划，可作校验或显示用）
- **Keil 工程配置**：地面站新增 `route_planning.c` 后需手动加入 Keil 工程编译列表（与之前 `uart_log.c`、`ano_device_usart3.c` 同样的注意事项）
- **内存预算**：地面站新增 9×7 动物地图（63 字节）、Route_Planning 模块静态 buffer（FlightPath ~400 字节 × 2 + grid 63 字节），总计约 1KB，STM32F407 SRAM 充裕
- **测试可执行性**：协议集成 seam 需要 FC↔Ground UART 连通，可在实验室环境用 USB 串口回环验证，不依赖实际飞行
