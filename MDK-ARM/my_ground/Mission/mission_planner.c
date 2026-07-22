/**
 * mission_planner.c — 地面站任务状态机
 *
 * 纯任务逻辑，不包含任何屏幕协议/UI 代码。
 * 屏幕相关功能由 touch_uart.c 通过 getter 函数获取数据后自行处理。
 *
 * 职责：
 * 1. 管理禁飞区 + zones_confirmed 标志
 * 2. 计算 + 缓存双路线（Patrol + Return）
 * 3. 响应 FC 握手相位（WAITING_PATROL / WAITING_RETURN）自动下发航点
 *
 * 架构原则：
 * - 计算与发送解耦：mission_request_route() 只算不发
 * - 分阶段下发：WAITING_PATROL 发 Patrol，WAITING_RETURN 发 Return
 * - mission 层不依赖 screen 层（单向依赖：screen → mission）
 */
#include "mission_planner.h"
#include "ano_device_ground.h"
#include "Route_Planning/route_planning.h"
#include "driver_registry.h"
#include "uart_log.h"
#include "uart_base.h"
#include "uarts.h"
#include "string.h"
#include <stdio.h>
#include "ano.h"
#include "touch_uart.h"
#define COM_DEBUG  1
#define CELL_SIZE  50   /* 网格边长 (cm)：world = grid * CELL_SIZE */


/* ================================================================
 *                        状态与缓存
 * ================================================================ */

/* ── 禁飞区（默认空集） ── */
static struct Point_index_t s_no_fly_zones[3];

/* ── 起点 ── */
static uint8_t s_start_x = 0;
static uint8_t s_start_y = 0;

/* ── 已发送航点相位锁存（0xFF = 从未发送） ── */
static uint8_t s_last_sent_phase = 0xFF;

/* ── 双路线缓存
 *
 * patrol/return_screen：压缩网格坐标（FlightPath）
 *   → 供 screen 层通过 getter 获取，用于屏幕预览
 * patrol/return_fc：完整世界坐标（Point_t[]）
 *   → 发给 FC 执行
 *
 * 两者同源：都从 plan_path() / plan_return_path() 派生
 */

static struct Point_t    s_patrol_to_fc_pst[200];;                             //int16_t 世界坐标格式
static struct FlightPath s_partrol_compress_st;

static struct Point_t    s_return_to_fc_pst[200];
static struct FlightPath s_return_compress_st;


/* ================================================================
 *                        内部辅助函数
 * ================================================================ */

uint8_t mission_get_fc_phase(void)
{
    struct gs_batt_curr_height_process_t st;
    batt_curr_height_process_copy(&st);
    return st.process_uc;
}

static bool mission_can_request_route(void)
{
    uint8_t phase = mission_get_fc_phase();
    return (phase == FC_PHASE_IDLE
         || phase == FC_PHASE_TAKEOFF_DELAY
         || phase == FC_PHASE_ALT_HOLD || phase == FC_PHASE_WAITING_PATROL);
}

static bool s_build_grid(uint8_t grid[WIDTH][HEIGHT])
{
    memset(grid, 0, WIDTH * HEIGHT);
    for (uint8_t i = 0; i < 3; i++) {
        if (s_no_fly_zones[i].x < WIDTH && s_no_fly_zones[i].y < HEIGHT && (s_no_fly_zones[i].x != 0 || s_no_fly_zones[i].y != 0)) {
            grid[s_no_fly_zones[i].x][s_no_fly_zones[i].y] = 1;                     //不超过索引，且s_no_fly_zone 不能给原点设置
        }
    }
    return is_block_horizontal(grid);
}

static void s_grid_to_world_array(const struct Point_index_t *grid_pts, uint8_t count,
                                  struct Point_t *world_pts)
{
    for (uint8_t i = 0; i < count; i++) {
        world_pts[i].x = (int16_t)(grid_pts[i].x * CELL_SIZE);
        world_pts[i].y = (int16_t)(grid_pts[i].y * CELL_SIZE);
    }
}


/* ================================================================
 *                     核心：计算 + 缓存
 * ================================================================ */

void mission_request_route(void)                                                           //由它来覆盖整个全局发送数组
{
    /* ── 1. 构建障碍物地图 ── */
    uint8_t grid[WIDTH][HEIGHT];
    bool horizontal = s_build_grid(grid);                                                  //先清零地图，往grid里面填充禁飞区坐标

    /* ── 2. 巡逻路径 ── */
    // struct Point_index_t full_path[MAX_PATH];
    struct FlightPath full_path_st = {0};
    //fwd_cnt 这里会是全部的航点数目
    
    full_path_st.count = plan_path(full_path_st.points, grid,                             //这是巡逻 plan_path
                                s_start_x, s_start_y, horizontal);                        

    compress_path(&full_path_st,&s_partrol_compress_st);

    s_grid_to_world_array(s_partrol_compress_st.points,s_partrol_compress_st.count,
                        s_patrol_to_fc_pst);                                             //full_path -- 整个巡逻路径

    /* * plan_return_path 读取 path[*index] 作为返航起点。
     * plan_path 返回 fwd_cnt = count，有效索引 0~fwd_cnt-1。
     * 所以 last_idx = fwd_cnt - 1（最后一个有效巡逻点）。 */

    uint8_t last_idx = (full_path_st.count > 0) ? (full_path_st.count - 1) : 0;         // 计算最后一个索引

    struct FlightPath back_raw_st = {0};

    plan_return_path(full_path_st.points, &last_idx, grid, last_idx, &back_raw_st);     // 第一个参数是整个巡逻的路线,第二个参数是当前路径的长度

    compress_path(&back_raw_st, &s_return_compress_st);

    s_grid_to_world_array(s_return_compress_st.points, s_return_compress_st.count, s_return_to_fc_pst);
}


bool mission_handle_request_route(void)
{
    if (!mission_can_request_route()) {
#if COM_DEBUG
        uart_printf_v(pstbase_screen_uart, 0, "[MSN] reject: phase %d\r\n",
            mission_get_fc_phase());
#endif
        return false;
    }

    //这个就是真正计算路径的函数，它会将
    mission_request_route();
    return true;
}



void mission_planner_init(void)
{
    s_last_sent_phase = 0xFF;
}
DRIVER_INIT(mission_planner_init);

/*
 * mission_planner_tick - 根据 FC 相位下发航点
 *
 * 采用电平触发 + 发送锁存：当 screen 已启动 且 FC 进入 WAITING 相位
 * 时发送对应航点，同一相位只发一次。解决了原边沿检测在屏幕未就绪
 * 时错过发送窗口的问题。
 */
//采用发送状态量， 发送后赋值一个状态量
void mission_planner_tick(void)
{
	uint8_t cur_phase;

	/* 屏幕未启动巡逻任务，不处理 */
	if (!screen_begin_fly_b())
		return;

	cur_phase = mission_get_fc_phase();

	/* 同一相位已发送过，跳过 */
	if (cur_phase == s_last_sent_phase)
		return;

	switch (cur_phase) {
	case FC_PHASE_WAITING_PATROL:
		ground_send_patrol_waypoints_v(s_patrol_to_fc_pst,
					       s_partrol_compress_st.count);
		screen_set_ui_mode(UI_MODE_PATROL);
		s_last_sent_phase = cur_phase;
#if COM_DEBUG
		uart_printf_v(pstbase_screen_uart, 0,
			      "[MSN] patrol waypoints sent\r\n");
#endif
		break;
	case FC_PHASE_WAITING_RETURN:
		ground_send_return_waypoints_v(s_return_to_fc_pst,
					       s_return_compress_st.count);
		screen_set_ui_mode(UI_MODE_PREVIEW);
		s_last_sent_phase = cur_phase;
#if COM_DEBUG
		uart_printf_v(pstbase_screen_uart, 0,
			      "[MSN] return waypoints sent\r\n");
#endif
		break;
	default:
		break;
	}
}


/* ================================================================
 *                      禁飞区管理
 * ================================================================ */

void mission_set_no_fly_zones(uint8_t count, struct Point_index_t* zones)
{
    if (count > 3) count = 3;
    for (uint8_t i = 0; i < count; i++) {
        s_no_fly_zones[i] = zones[i];
    }
#if COM_DEBUG
    uart_printf_v(pstbase_screen_uart, 0, "[MSN] zones: %d\r\n", count);
    // vano_printf(pstAnobase_Ground,ano_printf_green_em,"[MSN] zones: %d\r\n", count);
#endif
}

void mission_reset_no_fly_zones_v(void)
{
    memset(s_no_fly_zones, 0, sizeof(s_no_fly_zones));
}


/* ================================================================
 *                      Getter（供 screen 层调用）
 * ================================================================ */

void mission_copy_patrol_screen_path_v(struct FlightPath* out)
{
    *out = s_partrol_compress_st;                                   //发送给压缩后的点
}

void mission_copy_return_screen_path_v(struct FlightPath* out)
{
    *out = s_return_compress_st;                                   //发送给压缩后的点
}

