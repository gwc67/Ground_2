/**
 * @file route_planning.c
 * @brief 路径规划算法（从 FC 移植）
 *
 * 网格坐标系说明（右手定则）:
 *   - 原点: 左下角 (0,0) = 起飞点
 *   - X 轴 (grid_x = 列): 向右增加 (0→8)，+X 方向
 *   - Y 轴 (grid_y = 行): 向上增加 (0→6)，+Y 方向
 *   - 网格尺寸: 9 列 × 7 行 (WIDTH=9, HEIGHT=7)
 *
 * 弓字形扫描路径示意（水平扫描模式）:
 *   row=6 │  ↑   ← ← ← ← ← ← ← E  (X=8: 到达终点)
 *       5 │  ↑   → → → → → → → ↑
 *       4 │  ↑   ← ← ← ← ← ← ← ↑
 *       3 │  ↑   → → → → → → → ↑
 *       2 │  ↑   ← ← ← ← ← ← ← ↑
 *       1 │  ↑   → → → → → → → ↑
 *   row=0 │  S → → → → → → → ↑  (X=0: 从左向右，到达右边界后向上)
 *         └──────────────────────
 *        col=0  1  2  3  4  5  6  7  8  (列/+X)
 *
 * 坐标转换 (1-indexed):
 *   world_x = (grid_x ) * 50    // 列 × 50cm，范围 0~400
 *   world_y = (grid_y ) * 50    // 行 × 50cm，范围 0~300
 */

#include "route_planning.h"
#include <string.h>


//plan_mission 拿的是世界坐标计算

void grid_to_world(uint8_t grid_x, uint8_t grid_y, int16_t *world_x, int16_t *world_y)
{
    /* 新坐标系：grid_x=列(0~8) → world_x=0~400, grid_y=行(0~6) → world_y=0~300 */
    *world_x = (int16_t)(grid_x) * 50;
    *world_y = (int16_t)(grid_y) * 50;
}


void compress_path(struct FlightPath *original, struct FlightPath *compressed)
{
    if (original->count == 0) {
        compressed->count = 0;
        return;
    }

    compressed->count = 1;
    compressed->points[0] = original->points[0];

    if (original->count == 1) {
        return;
    }

    //问题，如果有很多拐点，那又是如何计算的呢？
    /* 确定初始方向 */
    int8_t dx = (int8_t)original->points[1].x - (int8_t)original->points[0].x;
    int8_t dy = (int8_t)original->points[1].y - (int8_t)original->points[0].y;
    int8_t last_dx = dx;
    int8_t last_dy = dy;

    for (uint8_t i = 2; i < original->count; i++) {
        dx = (int8_t)original->points[i].x - (int8_t)original->points[i - 1].x;
        dy = (int8_t)original->points[i].y - (int8_t)original->points[i - 1].y;

        /* 方向改变，记录上一个点 */
        if (dx != last_dx || dy != last_dy) {
            
            //这里会对压缩后的点进行计数自增操作
            compressed->points[compressed->count++] = original->points[i - 1];

            last_dx = dx;
            last_dy = dy;
        }
    }

   //无论什么算法，可以发现最后一个点永远无法计算到 
    /* 记录最后一个点 */
    compressed->points[compressed->count++] = original->points[original->count - 1];
}

/* ============== 障碍物检测 ============== */
// 检测网格地图中是否存在至少一组“连续 3 个水平相邻的障碍物”。
// 网格中 0 通常代表自由空间，1 代表障碍物（或禁飞区）
/* 那要是不是3格的 水平相邻的障碍物呢？ 竖直的呢*/
bool is_block_horizontal(uint8_t grid[WIDTH][HEIGHT])
{
    for (uint8_t y = 0; y < HEIGHT; y++) {
        for (uint8_t x = 0; x <= WIDTH - 3; x++) {
            if (grid[x][y] == 1 && grid[x + 1][y] == 1 && grid[x + 2][y] == 1) {
                return true;
            }
        }
    }
    return false;
}
// 这个函数 is_block_on_edge 的作用是判断障碍物是否紧贴地图边界。这是一个用于优化弓字形扫描航线规划的关键函数
// 输入：
// grid：网格地图（1 代表障碍物）
// horizontal：当前扫描方向是否为横向（水平）
// 输出：
// true：障碍物贴着边界
// false：障碍物未贴边界
// 1. 当 horizontal = true（横向扫描时）

static bool is_block_on_edge(uint8_t grid[WIDTH][HEIGHT], bool horizontal)
{
    if (horizontal) {
        /* 横向障碍物是否与上下边界重合 */
        // ：检查水平边界（上下边缘）
        for (uint8_t x = 0; x < WIDTH; x++) {
            if (grid[x][0] == 1 || grid[x][HEIGHT - 1] == 1) {
                return true;
            }
        }
        // 第二部分：检查垂直边界（左右边缘）的连续障碍物
        // 遍历所有行（y坐标）
        // 检查最左两列（x=0, x=1） 或最右两列（x=WIDTH-1, x=WIDTH-2） 是否同时有障碍物
        // 这是为了检测是否有厚度≥2的障碍物贴着左右边界
        for (uint8_t y = 0; y < HEIGHT; y++) {
            if ((grid[0][y] == 1 && grid[1][y] == 1) ||
                (grid[WIDTH - 1][y] == 1 && grid[WIDTH - 2][y] == 1)) {
                return true;
            }
        }
    } else {
        /* 纵向障碍物是否与左右边界重合 */

        for (uint8_t y = 0; y < HEIGHT; y++) {
            if (grid[0][y] == 1 || grid[WIDTH - 1][y] == 1) {
                return true;
            }
        }
        for (uint8_t x = 0; x < WIDTH; x++) {
            if ((grid[x][0] == 1 && grid[x][1] == 1) ||
                (grid[x][HEIGHT - 1] == 1 && grid[x][HEIGHT - 2] == 1)) {
                return true;
            }
        }
    }
    return false;
}

/// @brief 检测地图边界上是否存在连续3个或以上的障碍物，并返回具体是哪条边出现了这种情况。
/// @param grid 网格地图（1 表示障碍物）
/// @param horizontal 当前是否为横向扫描模式
/// @return
// 0：无特殊情况
// 1：底边有连续3个障碍物
// 2：顶边有连续3个障碍物
// 3：左边有连续3个障碍物
// 4：右边有连续3个障碍物
static uint8_t check_special_edge_case(uint8_t grid[WIDTH][HEIGHT], bool horizontal)
{
    if (horizontal) {
        for (uint8_t x = 0; x <= WIDTH - 3; x++) {
            if (grid[x][HEIGHT - 1] == 1 && grid[x + 1][HEIGHT - 1] == 1 && grid[x + 2][HEIGHT - 1] == 1) {
                return 1; /* 底边重合 */
            }
        }
        for (uint8_t x = 0; x <= WIDTH - 3; x++) {
            if (grid[x][0] == 1 && grid[x + 1][0] == 1 && grid[x + 2][0] == 1) {
                return 2; /* 顶边重合 */
            }
        }
    } else {
        for (uint8_t y = 0; y <= HEIGHT - 3; y++) {
            if (grid[0][y] == 1 && grid[0][y + 1] == 1 && grid[0][y + 2] == 1) {
                return 3; /* 左边重合 */
            }
        }
        for (uint8_t y = 0; y <= HEIGHT - 3; y++) {
            if (grid[WIDTH - 1][y] == 1 && grid[WIDTH - 1][y + 1] == 1 && grid[WIDTH - 1][y + 2] == 1) {
                return 4; /* 右边重合 */
            }
        }
    }
    return 0;
}

// 以上全部是grid 地图索引


/* ============== 路径规划辅助函数 ============== */

/// @brief 在满足条件的情况下，将一个新的坐标点添加到飞行路径中，并标记该点为已访问。
/// @param path 存储飞行路径的数组
/// @param index 当前路径数组中已存储点的数量（指针，用于修改外部变量
/// @param grid 网格地图
/// @param x 要添加的点的x坐标
/// @param y 要添加的点的y坐标
static void add_point(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT], uint8_t x, uint8_t y)
{
    // 第一步：检查边界和障碍物
    if (x < WIDTH && y < HEIGHT && grid[x][y] != 1) {
        grid[x][y] = 2;                     /* 标记为已访问 使用2 这个数字 */
        path[*index].x = x;                 // 第三步：将该点添加到路径数组中
        path[*index].y = y;
        (*index)++;                         // 第四步：更新路径计数器 递增路径计数器，指向下一个可用的存储位置
    }
}

/// 处理3个禁飞区在一起的函数
/// @brief 当无人机在横向（水平）扫描时，如果正前方遇到了一个宽度较小（比如 2~3 格）的孤立障碍物，它不会直接撞上去，也不会复杂地重新规划全局路径，而是执行一个固定的“U型”绕行动作，从障碍物旁边跨过去，然后继续原来的扫描方向。
/// @param path 
/// @param index 
/// @param grid 
/// @param x    
/// @param y    
/// @param dir  扫描方向	扫描方向。1 代表向右，-1 代表向左。 一共添加到了6个点
static void avoid_horizontal_center(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                     uint8_t *x, uint8_t *y, int8_t dir)
{
    // / 1. 向后退/偏移一行（假设 y-- 是向上偏移一行）
    (*y)--;
    add_point(path, index, grid, *x, *y);
    // 2. 平行移动 4 格，跨越障碍物
    for (uint8_t i = 0; i < 4; i++) {
        *x += dir;
        add_point(path, index, grid, *x, *y);
    }

    // 3. 回到原来的扫描行
    (*y)++;
    add_point(path, index, grid, *x, *y);
}

//为什么向右是加？ 取向上是正，取向右是正 地图原点应该取的是 最左下角
static void avoid_horizontal_edge_forward(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                           uint8_t *x, uint8_t *y, int8_t dir)
{
    // 1. 向后退/偏移一行（假设 y-- 是向下偏移一行）
    (*y)--;
    // 注意：这里没有立即 add_point，而是直接开始移动
    // 2. 平行移动 3 格，跨越障碍物
    for (uint8_t i = 0; i < 3; i++) {
        *x += dir;
        add_point(path, index, grid, *x, *y);
    }
    // 3. 回到原来的扫描行
    (*y)++;
     // 记录点 (x+3, y)
     add_point(path, index, grid, *x, *y);   
}



//        0 1 2 3 4 5 6 7 8  (X轴)
//      -------------------
//    0 | S → → → → → → → ↓  (0,0)→(8,0)→(8,1)
//    1 | ↑ ← ← ← ← ← ← ← ↓  (8,1)→(0,1)→(0,2)
//    2 | → → → → → → → [1]  (0,2)→(7,2) 【触发绕行】
//    3 | ← ← ← ← ← ← ↓ 0   (7,3)→(2,3)←←←←← 【avoid_horizontal_edge_turn】
//    4 | → → → → → → → → ↓  (2,4)→(8,4)→(8,5)  ※注：从(2,3)下到(2,4)后继续弓字
//    5 | ↑ ← ← ← ← ← ← ← ↓  (8,5)→(0,5)→(0,6)
//    6 | → → → → → → → → E  (0,6)→(8,6) 终点

// 根据弓字形扫描的铁律：
// 既然 y 行是向右的；
// 那么 y+1 行必须是向左的！
// 因此，当无人机向下变道（y++）进入 y+1 行后，它必须立刻开始向左扫描。
// 在代码中，向左移动就是 *x -= dir（因为当前 dir=1，减去 1 就是向左

/// @brief 放弃当前行，直接“掉头”进入下一行，并反转扫描方向
/// @param path  
/// @param index 
/// @param grid 
/// @param x 
/// @param y 
/// @param dir 
//这样的5个数据，不是还少了一列
static void avoid_horizontal_edge_turn(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                        uint8_t *x, uint8_t *y, int8_t dir)
{
    (*y)++;
    add_point(path, index, grid, *x, *y);
    for (uint8_t i = 0; i < 5; i++) {
        *x -= dir;
        add_point(path, index, grid, *x, *y);
    }
}

static void avoid_vertical_center(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                   uint8_t *x, uint8_t *y, int8_t dir)
{
    (*x)++;
    add_point(path, index, grid, *x, *y);
    for (uint8_t i = 0; i < 4; i++) {
        *y += dir;
        add_point(path, index, grid, *x, *y);
    }
    (*x)--;
    add_point(path, index, grid, *x, *y);
}


static void avoid_vertical_edge_forward(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                         uint8_t *x, uint8_t *y, int8_t dir)
{
    (*x)++;
    for (uint8_t i = 0; i < 3; i++) {
        *y += dir;
        add_point(path, index, grid, *x, *y);
    }
    (*x)--;
    add_point(path, index, grid, *x, *y);
}


static void avoid_vertical_edge_turn(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                      uint8_t *x, uint8_t *y, int8_t dir)
{
    (*x)--;
    add_point(path, index, grid, *x, *y);
    for (uint8_t i = 0; i < 3; i++) {
        *y -= dir;
        add_point(path, index, grid, *x, *y);
    }
}

static void avoid_special_edge_case(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                                     uint8_t *x, uint8_t *y, int8_t dir, uint8_t case_type)
{
    switch (case_type) {
    case 1: /* 横向底边重合 */
        (*y)--;
        add_point(path, index, grid, *x, *y);
        for (uint8_t i = 0; i < 4; i++) {
            *x += dir;
            add_point(path, index, grid, *x, *y);
        }
        (*y)++;
        add_point(path, index, grid, *x, *y);
        break;
    case 2: /* 横向顶边重合 */
        (*y)++;
        add_point(path, index, grid, *x, *y);
        for (uint8_t i = 0; i < 4; i++) {
            *x += dir;
            add_point(path, index, grid, *x, *y);
        }
        (*y)--;
        add_point(path, index, grid, *x, *y);
        break;
    case 3: /* 纵向左边重合 */
        (*x)++;
        add_point(path, index, grid, *x, *y);
        for (uint8_t i = 0; i < 4; i++) {
            *y += dir;
            add_point(path, index, grid, *x, *y);
        }
        (*x)--;
        add_point(path, index, grid, *x, *y);
        break;
    case 4: /* 纵向右边重合 */
        (*x)--;
        add_point(path, index, grid, *x, *y);
        for (uint8_t i = 0; i < 4; i++) {
            *y += dir;
            add_point(path, index, grid, *x, *y);
        }
        (*x)++;
        add_point(path, index, grid, *x, *y);
        break;
    }
}

/* ============== 核心路径规划 ============== */
// 理想情况（水平扫描模式）— 新坐标系，起点左下 (0,0):
//   row=6│  ↑   ← ← ← ← ← ← ← E  (row=6: 到达终点，右上)
//      5 │  ↑   → → → → → → → ↑
//      4 │  ↑   ← ← ← ← ← ← ← ↑
//      3 │  ↑   → → → → → → → ↑
//      2 │  ↑   ← ← ← ← ← ← ← ↑
//      1 │  ↑   → → → → → → → ↑
//   row=0│  S → → → → → → → ↑  (row=0: 从左向右，到达右边界后向上)
//        └──────────────────────
//       col=0  1  2  3  4  5  6  7  8  (列/+X 方向)
//
// 遇到障碍物时的绕行策略（grid 索引空间，与坐标系方向无关）:
//   - avoid_horizontal_center: 障碍物在中间，向相邻行绕行
//   - avoid_horizontal_edge_forward: 障碍物贴边，向前绕行
//   - avoid_horizontal_edge_turn: 障碍物贴边，掉头绕行
//   - avoid_special_edge_case: 特殊情况（连续 3 格贴边）

uint8_t plan_path(struct Point_index_t *path, uint8_t grid[WIDTH][HEIGHT],
                  uint8_t start_x, uint8_t start_y, bool horizontal)
{
    uint8_t index = 0, x = start_x, y = start_y;
    /* 新坐标系：起点在 col=0（左下），水平扫描先向右(+1)，垂直扫描先向上(+1) */
    int8_t dir = 1;

   
    bool edge = is_block_on_edge(grid, horizontal);
    //  检测地图边界上是否存在连续3个或以上的障碍物，并返回具体是哪条边出现了这种情况。
    uint8_t special_case = check_special_edge_case(grid, horizontal);
    
    //添加起点
    add_point(path, &index, grid, x, y);

    if (horizontal) {
        while (y < HEIGHT) {
            int8_t next_x = (int8_t)x + dir;                                                //一开始 dir = 1， 经过一轮 dir * -1 ； dir = -1 了  
            if (next_x >= 0 && next_x < WIDTH && grid[next_x][y] != 1) {
                x = (uint8_t)next_x;
                add_point(path, &index, grid, x, y);
            } else if (next_x >= 0 && next_x < WIDTH && grid[next_x][y] == 1) {
                if (special_case > 0) {
                    avoid_special_edge_case(path, &index, grid, &x, &y, dir, special_case);
                } else if (!edge) {
                    avoid_horizontal_center(path, &index, grid, &x, &y, dir);               //3个连续的禁飞区在中间              
                } else {
                    if (x == 0 || x == WIDTH - 1)
                        avoid_horizontal_edge_forward(path, &index, grid, &x, &y, dir);
                    else
                        avoid_horizontal_edge_turn(path, &index, grid, &x, &y, dir);             
                }
            } else {
                y++;
                if (y >= HEIGHT) break;
                add_point(path, &index, grid, x, y);                                        //添加拐点
                dir *= -1;                                                                  //当x扫完一行的时候往反方向触发
            }
        }
    } else {
        while ( x < WIDTH) {
            int8_t next_y = (int8_t)y + dir;
            if (next_y >= 0 && next_y < HEIGHT && grid[x][next_y] != 1) {
                y = (uint8_t)next_y;
                add_point(path, &index, grid, x, y);
            } else if (next_y >= 0 && next_y < HEIGHT && grid[x][next_y] == 1) {
                if (special_case > 0) {
                    avoid_special_edge_case(path, &index, grid, &x, &y, dir, special_case);
                } else if (!edge) {
                    avoid_vertical_center(path, &index, grid, &x, &y, dir);
                } else {
                    if (y == 0 || y == HEIGHT - 1)
                        avoid_vertical_edge_forward(path, &index, grid, &x, &y, dir);
                    else
                        avoid_vertical_edge_turn(path, &index, grid, &x, &y, dir);
                }
            } else {
                x++;  /* 新坐标系：从左往右换列 */
                if (x >= WIDTH) break;
                add_point(path, &index, grid, x, y);
                dir *= -1;
            }
        }
    }

    return index;
}

/* ============== 返航路径规划 ============== */

static bool is_path_clear(uint8_t grid[WIDTH][HEIGHT], uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    int8_t dx = (x1 > x0) ? 1 : (x1 < x0) ? -1 : 0;
    int8_t dy = (y1 > y0) ? 1 : (y1 < y0) ? -1 : 0;
    while (x0 != x1 || y0 != y1) {
        if (grid[x0][y0] == 1) return false;
        x0 += dx;
        y0 += dy;
    }
    return grid[x1][y1] != 1;
}

static void add_L_path(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                        uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    while (x0 != x1) {
        x0 += (x1 > x0) ? 1 : -1;
        add_point(path, index, grid, x0, y0);
    }
    while (y0 != y1) {
        y0 += (y1 > y0) ? 1 : -1;
        add_point(path, index, grid, x0, y0);
    }
}

//这里也转换里一下 -- 0 index线路
void plan_return_path(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                      uint8_t start_index, struct FlightPath *ret_fp)
{
    uint8_t x = path[*index].x;
    uint8_t y = path[*index].y;

    /* 新坐标系：起点 (0,0)=左下，巡逻终点 (8,6)=右上 */
    if (is_path_clear(grid, x, y, 0, 6) && is_path_clear(grid, 0, 6, 0, 0))  
    {
        /* 路线 1：沿顶边向左 → 沿左边向下回起点 */
        add_L_path(path, index, grid, x, y, 0, 6);                      //注意这里，它会在原来的path中进行追加，也就是在s_patrol_compute_st 中进行追加
        add_L_path(path, index, grid, 0, 6, 0, 0);                      //index++;
    } else {
        /* 路线 2：沿右边向下 → 沿底边向左回起点 */
        add_L_path(path, index, grid, x, y, 8, 0);
        add_L_path(path, index, grid, 8, 0, 0, 0);
    }

    ret_fp->count = 0;                                                  
    for (uint8_t i = start_index; i < *index; i++) {                    //返回的是full_path下重新追加的航点
        ret_fp->points[ret_fp->count].x = path[i].x;                    /* 保持 0-indexed */
        ret_fp->points[ret_fp->count].y = path[i].y;
        ret_fp->count++;
    }
}

/* ============== 完整任务规划 ============== */
// 输入：no_fly_zones 0-indexed (x=列, y=行)，start_x/start_y 0-indexed
// 这个函数返回巡逻点和返航点   
void plan_mission(struct Point_index_t no_fly_zones[3],
                  uint8_t start_x, uint8_t start_y,
                  struct Point_t *patrol_pts, uint8_t *patrol_cnt,
                  struct Point_t *return_pts, uint8_t *return_cnt)
{
    /* 1. 构建障碍物地图（输入 0-indexed，直接使用） */
    //每次是重新复位了
    uint8_t grid[WIDTH][HEIGHT] = {0};
    for (uint8_t i = 0; i < 3; i++) {
        if ( no_fly_zones[i].x < WIDTH &&
             no_fly_zones[i].y < HEIGHT) {
            grid[no_fly_zones[i].x][no_fly_zones[i].y] = 1;
        }
    }

    //如果禁飞区存在于起飞点，则将起飞点重新置0，刚好解决禁飞区必定位于原点的问题，起飞点必定不是0
    for (uint8_t i = 0; i < 3; i++)
    {
        if (no_fly_zones[i].x == 0 && no_fly_zones[i].y == 0)
        {
            grid[0][0] = 0;
        }
    }
    

    
    /* 2. 判断扫描方向 */
    bool horizontal = is_block_horizontal(grid);

    /* 3. 规划巡逻路径（网格坐标，0-indexed） */
    struct Point_index_t full_path[MAX_PATH];
    uint8_t index = plan_path(full_path, grid, start_x, start_y, horizontal);

    /* 4. 保持 0-indexed 并压缩 */
    struct FlightPath forward_raw;
    forward_raw.count = 0;
    for (uint8_t i = 0; i < index; i++) {
        forward_raw.points[i].x = full_path[i].x;
        forward_raw.points[i].y = full_path[i].y;
        forward_raw.count++;
    }

    struct FlightPath forward_compressed = {0};
    compress_path(&forward_raw, &forward_compressed);

    /* 5. 规划返航路径 */
    struct FlightPath back_raw = {0};
    plan_return_path(full_path, &index, grid, forward_raw.count, &back_raw);

    struct FlightPath back_compressed = {0};
    compress_path(&back_raw, &back_compressed);

   
    *patrol_cnt = forward_compressed.count;
    for (uint8_t i = 0; i < forward_compressed.count; i++) {
        grid_to_world(forward_compressed.points[i].x ,
                      forward_compressed.points[i].y ,
                      &patrol_pts[i].x, &patrol_pts[i].y);
    }

    *return_cnt = back_compressed.count;
    for (uint8_t i = 0; i < back_compressed.count; i++) {
        grid_to_world(back_compressed.points[i].x,
                      back_compressed.points[i].y,
                      &return_pts[i].x, &return_pts[i].y);
    }
}
