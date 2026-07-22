# Route_Planning — 网格路径规划算法（从 FC 移植）

## 概述

在 9×7 网格地图上为无人机规划弓字形（boustrophedon）巡逻路径和返航路径。支持禁飞区绕行、路径压缩（拐点提取）、网格→世界坐标转换。算法从飞控（FC）的 `Route_Planning.c` 移植到地面站，在地面站完成规划后将航点发送给飞控执行。

**核心能力：**
- 弓字形全覆盖扫描（水平 / 垂直两种模式）
- 禁飞区（最多 3 个网格点）自动绕行
- 共线中间点压缩，只保留拐点 + 起终点
- 返航路径规划（L 型折线，自动选择畅通路线）

## 坐标系

### 网格坐标

```
row=6│  (0,6) (1,6) (2,6) ... (8,6)  ← 顶
     │                              ↑ +Y
     │        飞行区域               │
     │                              │
row=0│  (0,0) (1,0) (2,0) ... (8,0)  ← 底=起飞点
     └──────────────────────────────┘
     col=0       → +X             col=8
```

- 原点：左下角 `(0, 0)` = 起飞点
- `col`（列，即 grid_x）：向右增加，范围 `0..8`（WIDTH=9）→ +X 方向
- `row`（行，即 grid_y）：向上增加，范围 `0..6`（HEIGHT=7）→ +Y 方向
- 右手定则：+X 向右，+Y 向上
- 内部计算用 **0-indexed**，对外接口（Mission 层、航点输出）用 **1-indexed**

### 世界坐标（cm）

```c
world_x = (grid_x - 1) * 50    // 列号 × 50cm，范围 0~400
world_y = (grid_y - 1) * 50    // 行号 × 50cm，范围 0~300
```

- 世界坐标原点 = 网格左下角（1-indexed 映射）
- +X 向右（列方向），+Y 向上（行方向）
- 飞控接收世界坐标（`struct Point_t`，`int16_t` 单位 cm）

## 弓字形扫描

### 水平扫描模式（默认）

```
       Y=0  1  2  3  4  5  6  7  8
     ┌─────────────────────────────┐
 X=0 │ S  →  →  →  →  →  →  →  ↓ │  向右扫到右边界，下行
   1 │ ↑  ←  ←  ←  ←  ←  ←  ←  ↓ │  向左扫到左边界，下行
   2 │ ↑  →  →  →  →  →  →  →  ↓ │  交替方向...
   3 │ ↑  ←  ←  ←  ←  ←  ←  ←  ↓ │
   4 │ ↑  →  →  →  →  →  →  →  ↓ │
   5 │ ↑  ←  ←  ←  ←  ←  ←  ←  ↓ │
   6 │ ↑  →  →  →  →  →  →  →  E │  终点
     └─────────────────────────────┘
```

- `dir` 初始 = 1（向右），每完成一行 `dir *= -1`
- 到边界（`next_x` 越界）→ 下行 `y++`，反转方向

### 垂直扫描模式

旋转 90°，按列扫描。由 `is_block_horizontal()` 判断启用哪种。

## 扫描方向判定

```c
bool is_block_horizontal(uint8_t grid[WIDTH][HEIGHT]);
```

检测是否存在**连续 3 个水平相邻**的障碍物。如果存在 → 使用垂直扫描（避开障碍长边）；否则 → 水平扫描。

> ⚠️ 判定逻辑比较特殊：名字是 `is_block_horizontal` 但返回 true 时选择**水平扫描**，实际语义是"障碍呈水平分布，适合沿水平方向扫描绕过"。

## 障碍物绕行策略

遇到前方格子是障碍物（`grid[next][y] == 1`）时，根据场景选择绕行方式：

### 优先级判定

```
前方有障碍？
  ├─ special_case > 0（连续 3+ 格贴边）→ avoid_special_edge_case()
  ├─ !edge（障碍不贴边界）            → avoid_horizontal_center()
  └─ edge（障碍贴边界）
       ├─ 当前位置在边界列（x==0 或 x==WIDTH-1）→ avoid_horizontal_edge_forward()
       └─ 否则                                    → avoid_horizontal_edge_turn()
```

### 各绕行函数示意

#### `avoid_horizontal_center` — 中部障碍，向上绕行

```
  扫描行 y-1:  ·  ·  ·  ·  ·  ·  ·  ·  ·   ← 向上偏移一行
  扫描行 y  :  →  →  [X] [X] [X]  ·  ·  ·  ·   ← 跨越 4 格障碍宽度
                ↑ 回原行
```

步骤：`y--` → 平行移动 4 格 → `y++` 回原行。共 6 个点。

#### `avoid_horizontal_edge_forward` — 贴边障碍，向前穿越

```
  扫描行 y-1:  ·  ·  ·  ·  ·  ·  ·  ·  ·   ← 偏移一行
  扫描行 y  :  [边界] [X] [X] [X]  ·  →  →  ·   ← 跨越 3 格 + 回到原行
```

步骤：`y--` → 平行 3 格 → `y++`。共 5 个点（不含起始 add_point）。

#### `avoid_horizontal_edge_turn` — 贴边障碍，掉头绕行

```
  扫描行 y  :  →  →  [X] [X] [X]  ·  ·  ·  ·
  扫描行 y+1:  ·  ←  ←  ←  ←  ←  ·  ·  ·  ·   ← 下行 + 反向扫 5 格
```

步骤：`y++` → 反向 `x -= dir` 移动 5 格。放弃当前行剩余部分。

#### `avoid_special_edge_case` — 连续 3+ 格贴边（特殊情况）

按 `case_type` 分 4 种方向：

| case_type | 含义 | 绕行方向 |
|-----------|------|---------|
| 1 | 横向底边贴边 | 向上绕（y--），平行 4 格，回原行 |
| 2 | 横向顶边贴边 | 向下绕（y++），平行 4 格，回原行 |
| 3 | 纵向左边贴边 | 向右绕（x++），平行 4 格，回原列 |
| 4 | 纵向右边贴边 | 向左绕（x--），平行 4 格，回原列 |

### 贴边检测

```c
static bool is_block_on_edge(uint8_t grid[WIDTH][HEIGHT], bool horizontal);
```

- `horizontal=true` 时检查：
  - 上下边界（`y==0` 或 `y==HEIGHT-1`）是否有障碍
  - 左右两列（`x==0/1` 或 `x==WIDTH-1/WIDTH-2`）是否有**厚度≥2**的障碍
- 任一条件满足 → 返回 true

```c
static uint8_t check_special_edge_case(uint8_t grid[WIDTH][HEIGHT], bool horizontal);
```

- 检查边界上是否存在**连续 3 格**障碍
- 返回值：0=无，1=底边，2=顶边，3=左边，4=右边

## 路径压缩

```c
void compress_path(struct FlightPath *original, struct FlightPath *compressed);
```

移除共线中间点，只保留**方向改变的拐点** + 起点 + 终点。

```
原始：  (0,0) → (1,0) → (2,0) → (3,0) → (3,1)
压缩后：(0,0) →              (3,0) → (3,1)
```

**算法：** 跟踪相邻点方向 `(dx, dy)`，方向变化时记录前一个点为拐点。循环结束后追加终点。

> ⚠️ 压缩后飞控仍需经过所有被压缩的格子 — 压缩只减少航点数量，不影响覆盖范围。

## 返航路径

```c
void plan_return_path(struct Point *path, uint8_t *index,
                      uint8_t grid[WIDTH][HEIGHT],
                      uint8_t start_index, struct FlightPath *ret_fp);
```

从巡逻终点返回起点 `(0, 0)`（网格左上角），使用 L 型折线沿边界行走：

```
路线 1（默认，路径畅通时）:
  终点 ──左转──→ (0, 6) ──右转──→ (8, 6) ──右转──→ (8, 0)
                    底边              右边              顶边到起点

路线 2（路线 1 被阻时）:
  终点 ──左转──→ (0, 0) ──右转──→ (8, 0)
                    左边              顶边到起点
```

- 路线选择通过 `is_path_clear()` 逐格检查是否有障碍
- 输出 `ret_fp` 中的坐标为 **1-indexed**
- 追加到 `path[]` 中，与巡逻路径共享数组

## 完整任务规划流程

```c
void plan_mission(struct Point no_fly_zones[3],
                  uint8_t start_x, uint8_t start_y,
                  struct Point_t *patrol_pts, uint8_t *patrol_cnt,
                  struct Point_t *return_pts, uint8_t *return_cnt);
```

```
输入：3 个禁飞区（1-indexed）+ 起点
  │
  ├─ 1. 构建障碍物地图 grid[9][7]（1-indexed → 0-indexed 填入 grid）
  ├─ 2. is_block_horizontal() → 决定扫描方向
  ├─ 3. plan_path() → 生成巡逻路径（0-indexed 网格坐标）
  ├─ 4. 转 1-indexed + compress_path() → 压缩巡逻路径
  ├─ 5. plan_return_path() → 追加返航路径 + 压缩
  └─ 6. grid_to_world() → 输出世界坐标（cm）
```

输出：`patrol_pts`（巡逻航点）+ `return_pts`（返航航点），均为 `struct Point_t`（世界坐标）。

## 辅助函数

| 函数 | 可见性 | 职责 |
|------|--------|------|
| `add_point()` | static | 添加点到路径 + 标记 grid 为已访问（`grid[x][y] = 2`） |
| `is_path_clear()` | static | 检查两点间直线是否无障碍 |
| `add_L_path()` | static | 生成 L 型折线路径（先走 X 再走 Y） |
| `is_block_on_edge()` | static | 检测障碍是否贴边界 |
| `check_special_edge_case()` | static | 检测连续 3+ 格贴边，返回边编号 |

## 依赖

| 头文件 | 用途 |
|-------|------|
| `Common/types.h` | `WIDTH`, `HEIGHT`, `MAX_PATH`, `struct Point`, `struct FlightPath`, `struct Point_t` |

**无其他模块依赖** — `Route_Planning` 是纯算法模块，不依赖 UART、Mission 等。反过来 Mission 层调用本模块。

## 文件清单

| 文件 | 职责 |
|------|------|
| `route_planning.h` | 公共接口声明 |
| `route_planning.c` | 全部算法实现（坐标转换、绕行、压缩、返航） |

## 核心规则

1. **内部 0-indexed，外部 1-indexed** — `plan_path()` 内部用 0-indexed 网格坐标，输出给 Mission 层前统一 +1 转 1-indexed。禁飞区输入则是 1-indexed，填入 grid 时 -1。
2. **grid 标记语义**：`0`=空闲，`1`=障碍物（禁飞区），`2`=已访问。`add_point()` 写入 2 后该格不会再被访问。
3. **`MAX_PATH = 200`** — 巡逻 + 返航共享同一个 `struct Point path[]` 数组（在 Mission 层分配），总点数不能超过 200。
4. **路径压缩是可选的** — 当前在 `plan_mission()` 中调用，但如果飞控需要逐格航点可跳过压缩。
5. **返航终点硬编码为 `(0, 0)`** — 路线 1 经过 `(0,6)→(8,6)→(8,0)`，路线 2 经过 `(0,0)→(8,0)`。起点固定是网格左上角。
6. **绕行函数中的坐标修改是副作用** — `avoid_*` 函数直接修改传入的 `*x`, `*y`，调用方在返回后继续用新坐标推进主循环。
7. **弓字形铁律** — 第 y 行向右，则第 y+1 行必须向左。`dir *= -1` 在每行结束时执行。

## 已知限制

1. **最多 3 个禁飞区** — `plan_mission` 只接受 `struct Point[3]`，更多禁飞区需改接口。
2. **绕行策略是启发式的** — 非全局最优路径规划（如 A*），而是固定模式的 U 型绕行。复杂障碍布局可能产生冗余路径。
3. **返航路线只有 2 种** — 路线 1（底→右→顶）和路线 2（左→顶），被阻时降级到路线 2。如果两条都被阻，仍会生成穿越障碍的路径。
4. **`is_block_horizontal()` 判定较粗糙** — 只看是否存在连续 3 个水平相邻障碍，不考虑整体分布。
