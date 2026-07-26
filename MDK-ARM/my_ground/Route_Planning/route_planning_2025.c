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

#define GRID_X  9
#define GRID_Y  7


static struct Point_2D_t s_no_fly_zone[3];  

static void s_point_3d_add_scan(struct point_3d_base* me,struct Point_3D_t* point_3d_pst)
{
    point_3d_pst->wp_action_uc = WP_ACTION_SCAN_em;
    point_3d_add_b(me,point_3d_pst);
}
static void s_point_3d_add_pass(struct point_3d_base* me,struct Point_3D_t* point_3d_pst)
{
    point_3d_pst->wp_action_uc = WP_ACTION_PASS_em;
    point_3d_add_b(me,point_3d_pst);
}

/// @brief 
/// @param in_pst    禁飞区数组
/// @param count_uc  0 ~ 2
void route_set_no_fly_zone(struct Point_2D_t * in_pst,uint8_t count_uc)
{
    count_uc = count_uc > 3 ? 3 : count_uc;

    for (uint8_t i = 0; i < count_uc; i++)
    {
        s_no_fly_zone[i] = in_pst[i];
    }
}


void route_reset_no_fly_zone(void)
{
    memset(s_no_fly_zone,0,sizeof(s_no_fly_zone));
}

static void no_fly_zone_to_grid(uint8_t grid_puc[GRID_Y][GRID_X] )
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (s_no_fly_zone[i].x_c != 0 || s_no_fly_zone[i].y_c != 0)
        {
            grid_puc[s_no_fly_zone[i].y_c][s_no_fly_zone[i].x_c] = 1;
        }
    }
}

static void s_add_map_point(struct Point_map_t* map_pst,struct Point_2D_t* point_2d_pst)
{
    map_pst->point_mat_pst[map_pst->count_uc++] = *point_2d_pst;
}
/**
 * @brief  判断连续3格障碍物的位置和朝向
 * 
 * @param  grid_puc  网格地图 (grid[y][x])
 * @return 障碍物位置枚举（含方向和贴边信息）
 */


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

                
                block_pst->x_c = dir_c > 0 ? x : x + 2;
                block_pst->y_c = y; 
            }

            /* ========== 垂直方向检测（连续3格在同一列） ========== */
            if (y <= GRID_Y - 3 &&
                grid_puc[y][x]     == 1 &&
                grid_puc[y + 1][x] == 1 &&
                grid_puc[y + 2][x] == 1)
            {
                /* 判断贴边情况 */
                if (x == 0)
                {
                    return BLOCK_LOC_VERTICAL_RIGHT_em;   /* 贴左边界，只能向右走 */
                }
                else if (x == GRID_X - 1)
                {
                    return BLOCK_LOC_VERTICAL_LEFT_em;    /* 贴右边界，只能向左走 */
                }
                else if (y == 0)
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
                block_pst->x_c = x;
                block_pst->y_c = dir_c > 0 ? y : y + 2;  
            }
        }
        dir_c *= -1;
    }

    return result;
}

// static void s_avoid_horizontal_center(struct Point_map_t* map_pst,int8_t* x_pc,int8_t* y_pc,int8_t dir_c)
// {

//     //水平模式下回到现在行
//     (*y_pc)--;
//     s_add_map_point(map_pst,*x_pc,*y_pc);

//     for (uint8_t i = 0; i < 4; i++)
//     {
//         *x_pc += dir_c;
//         s_add_map_point(map_pst,*x_pc,*y_pc);
//     }

//     //回到原来行
//     (*y_pc)++;
//     s_add_map_point(map_pst,*x_pc,*y_pc);
// }

//但是核心s_add_map_point 沿dir方向先跑满是没有问题的


// static void s_avoid_horizontal_edge_left_turn(struct Point_map_t* map_pst,int8_t* x_pc,int8_t* y_pc,int8_t dir_c)
// {
    // (*y_pc)
// }

static void s_block_loc_no_regular_plan(struct Point_map_t* map_pst);
static void s_block_loc_horizontal_center_plan(struct Point_map_t* map_pst ,struct Point_2D_t* block_st);
static void s_block_loc_horizontal_left_plan(struct Point_map_t* map_pst , struct Point_2D_t * block_st);

void plan_path_v(void)
{
    struct Point_map_t map_st = {0};

    uint8_t grid_puc[GRID_Y][GRID_X] = {0};  

    struct Point_2D_t block_st = {0};                                           //获取障碍物体的坐标
    
    no_fly_zone_to_grid(grid_puc);

    enum block_loc_e block_loc_em = get_block_loc_em(grid_puc,&block_st);

    block_loc_em = BLOCK_LOC_HORIZONTAL_LEFT_em;
    block_st.x_c = 2;
    block_st.y_c = 1;
    switch (block_loc_em)
    {
    case BLOCK_LOC_NO_REGULAR_em:           s_block_loc_no_regular_plan(&map_st);                         break;
    case BLOCK_LOC_HORIZONTAL_em:           s_block_loc_horizontal_center_plan(&map_st,&block_st);        break;
    case BLOCK_LOC_HORIZONTAL_LEFT_em :     s_block_loc_horizontal_left_plan(&map_st,&block_st);          break;
    default:
        break;
    }

    // char buf[30];

    for (int i = 0; i < map_st.count_uc; i++)
    {
        
        // uart_transmit(pstbase_screen_uart,(uint8_t*)buf,strlen(buf));
        // sprintf(buf,"(%d,%d)\r\n",map_st.point_mat_pst[i].x_c, map_st.point_mat_pst[i].y_c);

        uart_printf_v(pstbase_screen_uart, 0, "(%d,%d)\r\n", map_st.point_mat_pst[i].x_c, map_st.point_mat_pst[i].y_c);

        if (i == 58)
        {
            uart_printf_v(pstbase_screen_uart, 0, "为什么不打印(%d,%d)\r\n", map_st.point_mat_pst[i].x_c, map_st.point_mat_pst[i].y_c);
        }
        
    }
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
                // s_add_map_point(map_pst, &point_2d_st);
                // map_get_world_v(&point_2d_st, &point_3d_st);
                // s_point_3d_add_scan(g_patrol_point_3d_pst, &point_3d_st);

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








