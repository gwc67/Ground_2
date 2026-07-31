#include "route_planning_2025.h"
#include "map/map.h"
#include "Route_Planning/map/point_2d.h"
#include "string.h"
#include "point_3d.h"
#include "Ano_Scheduler.h"
#include "uarts.h"
#include "stdio.h"
#include "uart_log.h"
//因为传入的禁飞区是通过网格索引来进行的
//根据网格索引来制定航线的逻辑，这样可以减轻运算
//其实有没有padding不重要，只要保证接受和发送端一模一样即可

#define TEST_NEW 1


#define GRID_X  9
#define GRID_Y  7


static struct Point_2D_t s_no_fly_zone_st[3];  
static uint8_t s_grid_puc[GRID_Y][GRID_X];
#if TEST_NEW 

enum cell_state_e {
    CELL_FREE_em,
    CELL_BLOCKED_em,
    CELL_VISITED_em,
};


struct scan_cost_t {
    uint8_t segments;       // 连续FREE线段数量（越少越好）
    uint8_t blocked_hits;   // 碰到禁飞区的次数
} scan_cost_t;

static void s_evaluate_scan_axis(struct scan_cost_t *horiz,struct scan_cost_t *vert)
{
    *horiz = (struct scan_cost_t){0};
    *vert  = (struct scan_cost_t){0};

    // 评估水平方向：逐行统计
    for (uint8_t y = 0; y < GRID_Y; y++) {
        bool in_free_run = false;
        for (uint8_t x = 0; x < GRID_X; x++) {
            if (s_grid_puc[y][x] == CELL_FREE_em) {
                if (!in_free_run) { horiz->segments++; in_free_run = true; }
            } else {
                if (in_free_run) { horiz->blocked_hits++; in_free_run = false; }
            }
        }
    }

    // 评估垂直方向：逐列统计
    for (uint8_t x = 0; x < GRID_X; x++) {
        bool in_free_run = false;
        for (uint8_t y = 0; y < GRID_Y; y++) {
            if (s_grid_puc[y][x] == CELL_FREE_em) {
                if (!in_free_run) { vert->segments++; in_free_run = true; }
            } else {
                if (in_free_run) { vert->blocked_hits++; in_free_run = false; }
            }
        }
    }
}


static bool s_choose_horizontal_b(void)
{
    struct scan_cost_t h, v;
    s_evaluate_scan_axis(&h, &v);

    if (h.segments != v.segments)
        return h.segments < v.segments;

    // 线段数相同，选被阻挡更少的
    if (h.blocked_hits != v.blocked_hits)
        return h.blocked_hits < v.blocked_hits;

    // 完全相同，默认选较长轴（GRID_X=9 > GRID_Y=7 → 水平优先）
    return GRID_X >= GRID_Y;
}



/// @brief 
/// @param in_pst    禁飞区数组
/// @param count_uc  0 ~ 2
void route_set_no_fly_zone(struct Point_2D_t * in_pst,uint8_t count_uc)
{
    count_uc = count_uc > 3 ? 3 : count_uc;

    for (uint8_t i = 0; i < count_uc; i++)
    {
        s_no_fly_zone_st[i] = in_pst[i];
    }
}

void route_reset_no_fly_zone(void)
{
    memset(s_no_fly_zone_st,0,sizeof(s_no_fly_zone_st));
}

static void s_no_fly_zone_to_grid(void)
{
    memset(s_grid_puc,CELL_FREE_em,sizeof(s_grid_puc));
    for (uint8_t i = 0; i < 3; i++)
    {
        int8_t x_c = s_no_fly_zone_st[i].x_c;
        int8_t y_c = s_no_fly_zone_st[i].y_c;

        if ((x_c != 0 || y_c != 0)&& y_c < GRID_Y && x_c < GRID_X)
        {
            s_grid_puc[y_c][x_c] = CELL_BLOCKED_em;
        }
    }
}


static inline bool is_free_b(int8_t x_c,int8_t y_c)
{
    return (x_c < GRID_X && y_c < GRID_Y && s_grid_puc[y_c][x_c] == CELL_FREE_em);
}

static int s_add_way_point(struct Point_map_t* map_pst,struct point_3d_base* base,int8_t x_c ,int8_t y_c,enum wp_action_e action_em)
{
    if (x_c >= GRID_X || y_c >= GRID_Y )
    {
        return -1;
    }

    struct Point_2D_t p2d_st = {.x_c = x_c , .y_c = y_c};
    struct Point_3D_t p3d_st = {.z_s = 140,.wp_action_uc = action_em};
    map_get_world_v(&p2d_st,&p3d_st);

    if (map_pst->count_uc < sizeof(map_pst->point_mat_pst) / sizeof(map_pst->point_mat_pst[0]))
    {
        map_pst->point_mat_pst[map_pst->count_uc++] = p2d_st;
    }
    point_3d_add_b(base,&p3d_st);
    if (action_em == WP_ACTION_SCAN_em)
    {
        s_grid_puc[y_c][x_c] = CELL_VISITED_em;
    }
    return 0;
}
// true = 找到  false = 全覆盖完成
static bool find_remaining_free_b(int8_t* x_pc,int8_t* y_pc)
{
    for (uint8_t y = 0; y < GRID_Y; y++)
    {
        for (uint8_t x = 0; x < GRID_X; x++)
        {
            if (s_grid_puc[y][x] == CELL_FREE_em)
            {
                *x_pc = x;
                *y_pc = y;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 自适应蛇形扫描 — 在一条扫描线上遇到禁飞区时自动探测障碍范围，
 *        并从相邻行/列绕行，支持任意位置/形状的禁飞区。
 * @param horizontal_b true=水平扫描（逐行），false=垂直扫描（逐列）
 * @param primary     扫描线索引：水平时=行号(y)，垂直时=列号(x)
 * @param dir         扫描方向：1=正向，-1=反向
 */
static void s_scan_line(struct Point_map_t *map_pst, struct point_3d_base *base,
                        uint8_t primary, int8_t dir, bool horizontal_b)
{
    int8_t cur = (dir > 0) ? 0 : (int8_t)((horizontal_b ? GRID_X : GRID_Y) - 1);
    int8_t max_c = (int8_t)(horizontal_b ? GRID_X : GRID_Y);
    int8_t last_free = -1;

    while (cur >= 0 && cur < max_c) {
        int8_t gx = horizontal_b ? cur : (int8_t)primary;
        int8_t gy = horizontal_b ? (int8_t)primary : cur;

        if (is_free_b(gx, gy)) {
            /* 空闲格 → 正常扫描 */
            s_add_way_point(map_pst, base, gx, gy, WP_ACTION_SCAN_em);
            last_free = cur;
            cur += dir;
        } else {
            /* 遇到禁飞区 */
            if (last_free < 0) {
                break;  /* 本线首个格子即被阻塞，跳过整线 */
            }

            /* 向前探测，找到障碍后方第一个空闲格 */
            int8_t probe = cur + dir;
            while (probe >= 0 && probe < max_c) {
                int8_t px = horizontal_b ? probe : (int8_t)primary;
                int8_t py = horizontal_b ? (int8_t)primary : probe;
                if (is_free_b(px, py)) break;
                probe += dir;
            }
            if (probe < 0 || probe >= max_c) {
                break;  /* 后方全被阻挡，本线结束 */
            }

            /* 确定绕行邻线：优先 primary-1，越界则 primary+1 */
            int8_t adj_primary = (int8_t)primary;
            uint8_t adj_max = horizontal_b ? GRID_Y : GRID_X;
            if (primary > 0) {
                adj_primary = (int8_t)(primary - 1);
            } else if (primary + 1 < adj_max) {
                adj_primary = (int8_t)(primary + 1);
            } else {
                break;  /* 无可用的相邻线 */
            }

            /* —— 构建绕行路径 —— */
            struct Point_2D_t p2d;
            struct Point_3D_t p3d = {.z_s = 140, .wp_action_uc = WP_ACTION_PASS_em};

            /* 1) 在 last_free 位加 PASS（转向标记） */
            p2d.x_c = horizontal_b ? last_free : (int8_t)primary;
            p2d.y_c = horizontal_b ? (int8_t)primary : last_free;
            map_get_world_v(&p2d, &p3d);
            if (map_pst->count_uc < POINT_MAP_LENGTH)
                map_pst->point_mat_pst[map_pst->count_uc++] = p2d;
            point_3d_add_b(base, &p3d);

            /* 2) 移到邻线（同扫描轴偏移） */
            p2d.x_c = horizontal_b ? last_free : adj_primary;
            p2d.y_c = horizontal_b ? adj_primary : last_free;
            map_get_world_v(&p2d, &p3d);
            if (map_pst->count_uc < POINT_MAP_LENGTH)
                map_pst->point_mat_pst[map_pst->count_uc++] = p2d;
            point_3d_add_b(base, &p3d);

            /* 3) 在邻线上跨过障碍到 probe 位 */
            p2d.x_c = horizontal_b ? probe : adj_primary;
            p2d.y_c = horizontal_b ? adj_primary : probe;
            map_get_world_v(&p2d, &p3d);
            if (map_pst->count_uc < POINT_MAP_LENGTH)
                map_pst->point_mat_pst[map_pst->count_uc++] = p2d;
            point_3d_add_b(base, &p3d);

            /* 4) 回到原线，恢复扫描 */
            p2d.x_c = horizontal_b ? probe : (int8_t)primary;
            p2d.y_c = horizontal_b ? (int8_t)primary : probe;
            p3d.wp_action_uc = WP_ACTION_SCAN_em;
            map_get_world_v(&p2d, &p3d);
            if (map_pst->count_uc < POINT_MAP_LENGTH)
                map_pst->point_mat_pst[map_pst->count_uc++] = p2d;
            point_3d_add_b(base, &p3d);
            s_grid_puc[p2d.y_c][p2d.x_c] = CELL_VISITED_em;

            cur = probe + dir;
            last_free = probe;
        }
    }
}



void plan_path_new(void)
{
    struct Point_map_t map_st = {0};
    struct point_3d_base *base = g_patrol_point_3d_pst;

    s_no_fly_zone_to_grid();

    bool is_horizontal_b = s_choose_horizontal_b();
    uint8_t line_count = is_horizontal_b ? GRID_Y : GRID_X;
    int8_t dir_c = 1;

    for (uint8_t i = 0; i < line_count; i++) {
        s_scan_line(&map_st, base, i, dir_c, is_horizontal_b);
        dir_c *= -1;
    }

    /* 处理可能被障碍隔绝的孤立空闲格 */
    int8_t rem_x, rem_y;
    while (find_remaining_free_b(&rem_x, &rem_y)) {
        s_add_way_point(&map_st, base, rem_x, rem_y, WP_ACTION_SCAN_em);
    }
}


#else

/// @brief block_pst 是触发点的block_pst x,y 坐标，对于 dir = 1 ，触发点就是 索引号 ，对于dir = - 1 ， 触发的 索引号应该 + 2
static enum block_loc_e get_block_loc_em(uint8_t grid_puc[GRID_Y][GRID_X],struct Point_2D_t* block_pst)
{
    enum block_loc_e result = BLOCK_LOC_NO_REGULAR_em;

    int8_t dir_c = 1;
    /* 遍历所有可能的起始点 */
    for (uint8_t y = 0; y < GRID_Y; y++)
    {
        for (uint8_t x = 0; x < GRID_X; x++)
        {
            /* ========== 水平方向检测（连续3格在同一行） ========== */
            if (x <= GRID_X - 3 &&
                grid_puc[y][x]     == 1 &&
                grid_puc[y][x + 1] == 1 &&
                grid_puc[y][x + 2] == 1)
            {
                block_pst->x_c = dir_c > 0 ? x : x + 2;
                block_pst->y_c = y; 
                 if (x == 0)
                {
                    return BLOCK_LOC_HORIZONTAL_LEFT_em;  /* 贴左边界 */
                }
                else if (x + 2 == GRID_X - 1)
                {
                    return BLOCK_LOC_HORIZONTAL_RIGHT_em; /* 贴右边界 */
                }
                else
                {
                    /* 内部水平障碍物，记录但不立即返回，继续搜索是否有贴边的 */
                    result = BLOCK_LOC_HORIZONTAL_em;
                }
            }
        }
        dir_c *= -1;
    }

    dir_c = 1;
    for (uint8_t x = 0; x < GRID_X; x++)
    {
        for (uint8_t y = 0; y < GRID_Y; y++)
        {
            /* ========== 垂直方向检测（连续3格在同一列） ========== */
            if (y <= GRID_Y - 3 &&
                grid_puc[y][x]     == 1 &&
                grid_puc[y + 1][x] == 1 &&
                grid_puc[y + 2][x] == 1)
            {
                block_pst->x_c = x;
                block_pst->y_c = dir_c > 0 ? y : y + 2;
                /* 判断贴边情况 */
                 if (y == 0)
                {
                    return BLOCK_LOC_VERTICAL_DOWN_em;    /* 贴上边界 */
                }
                else if (y + 2 == GRID_Y - 1)
                {
                    return BLOCK_LOC_VERTICAL_UP_em;      /* 贴下边界 */
                }
                else
                {
                    /* 内部垂直障碍物 */
                    result = BLOCK_LOC_VERTICAL_em;
                }
            }
        }
        dir_c *= -1;
    }
    return result;
}


static void s_block_loc_no_regular_plan(struct Point_map_t* map_pst);

static void s_block_loc_horizontal_center_plan(struct Point_map_t* map_pst ,struct Point_2D_t* block_st);
static void s_block_loc_horizontal_left_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st);
static void s_block_loc_horizontal_right_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st);

static void s_block_loc_vertical_center_plan(struct Point_map_t* map_pst ,struct Point_2D_t* block_st);
static void s_block_loc_vertical_up_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st);
static void s_block_loc_vertical_down_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st);


void plan_path_v(void)
{
    struct Point_map_t map_st = {0};

    uint8_t grid_puc[GRID_Y][GRID_X] = {0};  

    struct Point_2D_t block_st = {0};                                           //获取障碍物体的坐标

#if PLAN_TEST

    struct Point_2D_t test[3] = {
        [0] = {.x_c = 2,.y_c = 2},
        [1] = {.x_c = 2,.y_c = 1},
        [2] = {.x_c = 2,.y_c = 0},
    };
    route_set_no_fly_zone(test,3);
#endif
    no_fly_zone_to_grid(grid_puc);

    enum block_loc_e block_loc_em = get_block_loc_em(grid_puc,&block_st);

    switch (block_loc_em)
    {
    case BLOCK_LOC_NO_REGULAR_em:           s_block_loc_no_regular_plan(&map_st);                           break;
    case BLOCK_LOC_HORIZONTAL_em:           s_block_loc_horizontal_center_plan(&map_st,&block_st);          break;
    case BLOCK_LOC_HORIZONTAL_LEFT_em :     s_block_loc_horizontal_left_plan(&map_st,&block_st);            break;
    case BLOCK_LOC_HORIZONTAL_RIGHT_em :    s_block_loc_horizontal_right_plan(&map_st,&block_st);           break;

    case BLOCK_LOC_VERTICAL_em:             s_block_loc_vertical_center_plan(&map_st,&block_st);            break;
    case BLOCK_LOC_VERTICAL_DOWN_em:        s_block_loc_vertical_down_plan(&map_st,&block_st);             break;
    case BLOCK_LOC_VERTICAL_UP_em:          s_block_loc_vertical_up_plan(&map_st,&block_st);              break;
    default:
        break;
    }
#if PLAN_TEST
    for (int i = 0; i < map_st.count_uc; i++)
    {        
        uart_printf_v(pstbase_screen_uart, 0, "(%d,%d),\r\n", map_st.point_mat_pst[i].x_c, map_st.point_mat_pst[i].y_c);
    }
#endif
}

//没有障碍物
static void s_block_loc_no_regular_plan(struct Point_map_t* map_pst)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};


    
    int8_t dir_c = 1;
    s_add_map_point(map_pst,&point_2d_st);  //先添加原点（0，0）

    for (int y = 0; y < GRID_Y ; y++)
    {
        for (int x = 0; x < GRID_X; x++)
        {
            point_2d_st.x_c += dir_c;
            s_add_map_point(map_pst,&point_2d_st);
            map_get_world_v(&point_2d_st,&point_3d_st);
            s_point_3d_add_scan(g_patrol_point_3d_pst,&point_3d_st);
        }
        point_2d_st.y_c = y;

        dir_c *= -1;
    }
    
    point_2d_st.x_c = 0;
    point_2d_st.y_c = 0;
    s_add_map_point(map_pst,&point_2d_st);
    map_get_world_v(&point_2d_st,&point_3d_st);
    s_point_3d_add_pass(g_patrol_point_3d_pst,&point_3d_st);
}

static void s_block_loc_horizontal_center_plan(struct Point_map_t* map_pst ,struct Point_2D_t* block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};

    int8_t dir_c = 1;

    for (int y = 0; y < GRID_Y; y++)
    {
    
        point_2d_st.y_c = y;

        for (int x = 0; x < GRID_X;)
        {
            point_2d_st.x_c = dir_c > 0 ? x : GRID_X -x - 1; 
            
            //使用的运动的那端的索引
            if (point_2d_st.x_c == block_st->x_c - dir_c && point_2d_st.y_c== block_st->y_c)
            {
                
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                if (y > 0)                                  //NB  point_2d_st.y_c = y; 这个真是十分巧妙
                {
                    point_2d_st.y_c = y - 1;
                }
                else if (y == 0)
                {
                    point_2d_st.y_c = y + 1;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                for (int i = 0; i < 4; i++)
                {
                    point_2d_st.x_c += dir_c;
                       
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.y_c = y;

                // 1. 显式添加回归 Scan 点（不再依赖 else 意外补全）
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);

                // 2. 用物理坐标反算替代硬编码的 x += 4                             //由于x += 4 的时候 ，st.x = 1 巧合会在下一个else里面执行，导致重复了一下
                int next_x_c = point_2d_st.x_c + dir_c;
                if (next_x_c >= 0 && next_x_c < GRID_X)
                    x = (dir_c > 0) ? next_x_c : (GRID_X - 1 - next_x_c);
                else
                    x = GRID_X; // 超出边界，直接结束本行扫描
            }
            else
            {
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                s_add_map_point(map_pst, &point_2d_st);
                x++;
            }
        }
        dir_c *= -1;
    }
}


static void s_block_loc_horizontal_left_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};

    int8_t dir_c = 1;

    for (int8_t y = 0; y < GRID_Y; y++)
    {
        point_2d_st.y_c = y;
        for (int8_t x = 0; x < GRID_X;)
        {
            point_2d_st.x_c = dir_c > 0 ? x : GRID_X - x - 1; 

            //block_st 处于 dir = 1 的 方向
            if (block_st->x_c == 0 && block_st->y_c == point_2d_st.y_c && point_2d_st.x_c == 0)
            {
                //由于这个到达这里的时候y 已经自增一次了，其实直接减掉就行
                point_2d_st.y_c = y - 1;

                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.x_c += dir_c;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                
                point_2d_st.y_c = y;

                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);    
                int next_x_c = point_2d_st.x_c + dir_c;
                if (next_x_c >= 0 && next_x_c < GRID_X)
                    x = (dir_c > 0) ? next_x_c : (GRID_X - 1 - next_x_c);
                else
                    x = GRID_X; // 超出边界，直接结束本行扫描
                
            }
            else if (dir_c == -1 && point_2d_st.x_c == block_st->x_c + 1 && point_2d_st.y_c == block_st->y_c)
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.y_c += 1;
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st); 
                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.x_c += dir_c;
                }
                int next_x_c = point_2d_st.x_c + dir_c;
                if (next_x_c >= 0 && next_x_c < GRID_X)
                    x = (dir_c > 0) ? next_x_c : (GRID_X - 1 - next_x_c);
                else
                    x = GRID_X; // 超出边界，直接结束本行扫描
            }
            else
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                x++;
            }
            
        }
        dir_c *= -1;
    }
    
}

static void s_block_loc_horizontal_right_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};


    int8_t dir_c = 1;
    

     for (int8_t y = 0; y < GRID_Y; y++)
    {
        point_2d_st.y_c = y;
        for (int8_t x = 0; x < GRID_X;)
        {
            point_2d_st.x_c = dir_c > 0 ? x : GRID_X - x - 1; 

            //block_st 处于 dir = -1 的 方向
            if (block_st->x_c == GRID_X - 1 && block_st->y_c == point_2d_st.y_c && point_2d_st.x_c == GRID_X - 1)
            {
                //由于这个到达这里的时候y 已经自增一次了，其实直接减掉就行
                point_2d_st.y_c = y - 1;

                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.x_c += dir_c;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                
                point_2d_st.y_c = y;

                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);    
                int next_x_c = point_2d_st.x_c + dir_c;
                if (next_x_c >= 0 && next_x_c < GRID_X)
                    x = (dir_c > 0) ? next_x_c : (GRID_X - 1 - next_x_c);
                else
                    x = GRID_X; // 超出边界，直接结束本行扫描
                
            }
            else if (dir_c == 1 && point_2d_st.x_c == block_st->x_c - 1 && point_2d_st.y_c == block_st->y_c)
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.y_c += 1;
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st); 
                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.x_c += dir_c;
                }
                int next_x_c = point_2d_st.x_c + dir_c;
                if (next_x_c >= 0 && next_x_c < GRID_X)
                    x = (dir_c > 0) ? next_x_c : (GRID_X - 1 - next_x_c);
                else
                    x = GRID_X; // 超出边界，直接结束本行扫描
            }
            else
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                x++;
            }
            
        }
        dir_c *= -1;
    }
    
}



static void s_block_loc_vertical_center_plan(struct Point_map_t* map_pst ,struct Point_2D_t* block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};

    int8_t dir_c = 1;

    for (int x = 0; x < GRID_X; x++)
    {
    
        point_2d_st.x_c = x;

        for (int y = 0; y < GRID_Y;)
        {
            point_2d_st.y_c = dir_c > 0 ? y : GRID_Y -y - 1; 
            
            //使用的运动的那端的索引
            if (point_2d_st.y_c == block_st->y_c - dir_c && point_2d_st.x_c == block_st->x_c)
            {
                
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                if ( block_st->x_c > 0)                                  //NB  point_2d_st.a_c = a; 这个真是十分巧妙
                {
                    point_2d_st.x_c = x - 1;
                }
                else if ( block_st->x_c == 0)
                {
                    point_2d_st.x_c = x + 1;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                for (int i = 0; i < 4; i++)
                {
                    point_2d_st.y_c += dir_c;
                       
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.x_c = x;

                // 1. 显式添加回归 Scan 点（不再依赖 else 意外补全）
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);

                // 2. 用物理坐标反算替代硬编码的 y += 4                             //由于y += 4 的时候 ，st.y = 1 巧合会在下一个else里面执行，导致重复了一下
                int neyt_y_c = point_2d_st.y_c + dir_c;
                if (neyt_y_c >= 0 && neyt_y_c < GRID_Y)
                    y = (dir_c > 0) ? neyt_y_c : (GRID_Y - 1 - neyt_y_c);
                else
                    y = GRID_Y; // 超出边界，直接结束本行扫描
            }
            else
            {
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                s_add_map_point(map_pst, &point_2d_st);
                y++;
            }
        }
        dir_c *= -1;
    }
}

static void s_block_loc_vertical_up_plan(struct Point_map_t *map_pst, struct Point_2D_t *block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};

    int8_t dir_c = 1;

    for (int8_t x = 0; x < GRID_X; x++)
    {
        point_2d_st.x_c = x;
        for (int8_t y = 0; y < GRID_Y;)
        {
            point_2d_st.y_c = dir_c > 0 ? y : GRID_Y - y - 1;

            // block_st 处于 dir = -1 的 方向
            if (block_st->y_c == GRID_Y - 1 && block_st->x_c == point_2d_st.x_c && point_2d_st.y_c == GRID_Y - 1)
            {
                // 由于这个到达这里的时候x 已经自增一次了，其实直接减掉就行
                point_2d_st.x_c = x - 1;

                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.y_c += dir_c;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.x_c = x;

                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                int next_y_c = point_2d_st.y_c + dir_c;
                if (next_y_c >= 0 && next_y_c < GRID_Y)
                    y = (dir_c > 0) ? next_y_c : (GRID_Y - 1 - next_y_c);
                else
                    y = GRID_Y; // 超出边界，直接结束本列扫描
            }
            else if (dir_c == 1 && point_2d_st.y_c == block_st->y_c - 1 && point_2d_st.x_c == block_st->x_c)
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.x_c += 1;
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.y_c += dir_c;
                }
                int next_y_c = point_2d_st.y_c + dir_c;
                if (next_y_c >= 0 && next_y_c < GRID_Y)
                    y = (dir_c > 0) ? next_y_c : (GRID_Y - 1 - next_y_c);
                else
                    y = GRID_Y; // 超出边界，直接结束本列扫描
            }
            else
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                y++;
            }
        }
        dir_c *= -1;
    }
}
static void s_block_loc_vertical_down_plan(struct Point_map_t *map_pst, struct Point_2D_t *block_st)
{
    struct Point_3D_t point_3d_st = {.z_s = 140};
    struct Point_2D_t point_2d_st = {0};

    int8_t dir_c = 1;

    for (int8_t x = 0; x < GRID_X; x++)
    {
        point_2d_st.x_c = x;
        for (int8_t y = 0; y < GRID_Y;)
        {
            point_2d_st.y_c = dir_c > 0 ? y : GRID_Y - y - 1;

            // block_st 处于 dir = 1 的 方向
            if (block_st->y_c == 0 && block_st->x_c == point_2d_st.x_c && point_2d_st.y_c == 0)
            {
                // 由于这个到达这里的时候x 已经自增一次了，其实直接减掉就行
                point_2d_st.x_c = x - 1;

                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.y_c += dir_c;
                }
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.x_c = x;

                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                int next_y_c = point_2d_st.y_c + dir_c;
                if (next_y_c >= 0 && next_y_c < GRID_Y)
                    y = (dir_c > 0) ? next_y_c : (GRID_Y - 1 - next_y_c);
                else
                    y = GRID_Y; // 超出边界，直接结束本列扫描
            }
            else if (dir_c == -1 && point_2d_st.y_c == block_st->y_c + 1 && point_2d_st.x_c == block_st->x_c)
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);

                point_2d_st.x_c += 1;
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_pass(g_patrol_point_3d_pst, &point_3d_st);
                for (int i = 0; i < 3; i++)
                {
                    point_2d_st.y_c += dir_c;
                }
                int next_y_c = point_2d_st.y_c + dir_c;
                if (next_y_c >= 0 && next_y_c < GRID_Y)
                    y = (dir_c > 0) ? next_y_c : (GRID_Y - 1 - next_y_c);
                else
                    y = GRID_Y; // 超出边界，直接结束本列扫描
            }
            else
            {
                s_add_map_point(map_pst, &point_2d_st);
                map_get_world_v(&point_2d_st, &point_3d_st);
                s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);
                y++;
            }
        }
        dir_c *= -1;
    }
}

#endif
