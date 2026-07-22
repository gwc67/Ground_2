# Touch Screen Protocol — 真实串口屏 HMI 交互

## Problem Statement

当前地面站的 HMI 交互依赖 Python 上位机通过 USART2 发送 ASCII 命令（`MAP:`, `REQUEST_PATH`, `REQUEST_RETURN`）进行调试。生产环境中，Python 上位机将被真实串口触摸屏取代。触摸屏有独立的显示和触摸输入能力，需要一套自定义协议与 STM32 地面站固件通信：用户可在屏幕上点击设置禁飞区、请求路径预览、请求返航预览，STM32 将规划结果以分包形式发回屏幕显示。

## Solution

在 `touch_uart.c` 中实现编译期可切换的双协议解析层：`PYTHON_DEBUG=1` 时走 Python 上位机协议（已有），`PYTHON_DEBUG=0` 时走真实串口屏协议（新增）。两层共用同一个公共接口 `screen_uart_check()`，遵循 Linux 风格的分离关注点架构——协议解析层只做解析和调度，不持有任何业务状态；路径规划和航点发送下沉到 `Mission/mission_planner.c`。

触摸屏仅用于**预览**路径，不直接触发飞控执行。飞控的巡逻/返航由 FC 自身相位状态机触发，屏幕预览与 FC 执行完全解耦。

## User Stories

1. As a 操作员, I want to 在触摸屏上点击网格格子设置禁飞区, so that 我可以直观地标注飞行区域中的障碍物
2. As a 操作员, I want to 一次设置最多 3 个禁飞区, so that 符合飞控的路径规划算法限制
3. As a 操作员, I want to 设置完禁飞区后在屏幕上看到规划出的巡逻路径, so that 我可以在起飞前确认路径是否合理
4. As a 操作员, I want to 在屏幕上看到返航路径, so that 我可以确认无人机完成任务后的返回路线
5. As a 操作员, I want to 路径预览不影响飞控的当前状态, so that 预览操作不会意外触发无人机起飞或返航
6. As a 操作员, I want to 在 Python 调试模式和真实屏幕模式之间通过编译开关切换, so that 开发和生产环境共用同一套代码库
7. As a 地面站固件, I want to 将路径分包发送（每包 ≤10 个航点）, so that 不超出屏幕端的接收缓冲区限制
8. As a 地面站固件, I want to 路径分包带结束标记, so that 屏幕端知道何时停止累积并绘制完整路径
9. As a 地面站固件, I want to 禁飞区坐标直接覆盖（非增量）, so that 每次设置都是确定性的，不存在状态残留
10. As a 地面站固件, I want to 使用 0-indexed 网格坐标作为协议坐标, so that 与内部算法坐标系一致，无需转换

## Implementation Decisions

### 协议设计

**屏幕 → STM32（上行命令）：**

| 命令 | 格式 | 含义 |
|---|---|---|
| 禁飞区确认 | `zone:x1,y1,x2,y2,...\n` | 覆盖当前禁飞区（0-indexed，最多 3 个 = 6 个坐标值） |
| 请求巡逻路径 | `request_route\n` | 触发巡逻路径预览 |
| 请求返航路径 | `return_route\n` | 触发返航路径预览 |

**STM32 → 屏幕（下行数据）：**

| 命令 | 格式 | 含义 |
|---|---|---|
| 巡逻路径分包 | `path:x1,y1,x2,y2,...\n` | 压缩后 0-indexed 网格坐标，每行 ≤10 个点 |
| 巡逻路径结束 | `path_end\n` | 屏幕停止累积，绘制完整巡逻路径 |
| 返航路径分包 | `return:x1,y1,x2,y2,...\n` | 同上格式 |
| 返航路径结束 | `return_end\n` | 屏幕停止累积，绘制完整返航路径 |

### 坐标系

全链路统一 **0-indexed**：

- 网格坐标：列 0~8，行 0~6
- 原点：左下角 (0,0) = 起飞点
- 右手定则：+X 向右（列），+Y 向上（行）
- `grid_to_world()`：`world = grid * 50`（无偏移）
- `world_pt_to_grid()`：`grid = world / 50`（无偏移）
- `plan_path()` 输入/输出均为 0-indexed
- `plan_return_path()` 输出 0-indexed
- `s_no_fly_zones` 存储 0-indexed
- `screen` 协议收发均为 0-indexed，直接透传无转换

### 架构

**编译期切换（Linux Kconfig 模式）：**

```
touch_uart.c:
  #if PYTHON_DEBUG
    screen_uart_check_python()   // 已有，不动
  #else
    screen_uart_check_touch()    // 新增，真实屏幕协议
  #endif

DrvUartDataCheck():
  #if PYTHON_DEBUG
    screen_uart_check_python();
  #else
    screen_uart_check_touch();
  #endif
```

**分层职责：**

| 层 | 模块 | 职责 |
|---|---|---|
| 协议解析 | `touch_uart.c` | 行组装 + 命令分派，不持有任何状态 |
| 任务调度 | `mission_planner.c` | 路径规划 + 航点发送 + 状态管理 |
| 路径算法 | `route_planning.c` | 纯算法，无 UART 依赖 |

**预览与执行分离：**

- `request_route` / `return_route`：只发屏幕，不发飞控，不改 `s_mission_state`
- FC 状态机（`FC_PHASE_PATROL` / `FC_PHASE_RETURN`）：独立触发，发飞控，改状态

### 路径分包规格

- 每行最多 10 个航点（约 55 字节，屏幕 128 字节缓冲区充裕）
- 压缩后的路径点（拐点），非逐格点
- 分包前缀：巡逻用 `path:`，返航用 `return:`
- 结束标记：`path_end` / `return_end`

### 新增函数

- `mission_screen_request_patrol()`：规划巡逻路径，分包发送 `path:` + `path_end` 到屏幕，不发飞控
- `mission_screen_request_return()`：规划返航路径，分包发送 `return:` + `return_end` 到屏幕，不发飞控
- 内部调用 `plan_path()` + `compress_path()` + `plan_return_path()` 获取 0-indexed 网格坐标

## Testing Decisions

- **协议解析测试**：通过 Python 上位机发送 `zone:`、`request_route`、`return_route` 命令，验证 STM32 正确解析并响应
- **分包完整性测试**：发送超过 10 个点的路径，验证屏幕端能正确累积并识别 `path_end`
- **坐标一致性测试**：设置禁飞区 (3,4)，规划路径，验证路径绕行该格子
- **预览隔离测试**：执行 `request_route` 后，验证 FC 状态未改变、航点未发送
- **模式切换测试**：分别以 `PYTHON_DEBUG=1` 和 `PYTHON_DEBUG=0` 编译，验证两种模式独立工作

## Out of Scope

- 飞控航点发送逻辑（已由 FC 状态机处理）
- 屏幕端 UI 渲染和触摸检测（屏幕固件负责）
- 航点分批发送（0x16 协议，已在 `ano_device_ground.c` 中实现）
- 坐标系转换的运行时切换（已硬编码为 0-indexed）
- 注释修正（6 处过时注释已标记，不在本 PRD 范围内）

## Further Notes

- 屏幕端行缓冲区 128 字节，每行 `path:` 数据 ≤55 字节，留有余量
- `plan_mission()` 中的注释（第 545 行）仍写着"grid_to_world 需要 1-indexed，所以 +1"，但代码已改为 0-indexed 直接传入，注释过时
- `route_planning.h` 中的函数注释仍标注 1-indexed，需后续更新
- `s_no_fly_zones` 默认值 `{3,1},{3,2},{3,3}` 是 0-indexed，与新坐标系一致
