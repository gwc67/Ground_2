#ifndef __MISSION_PLANNER_H
#define __MISSION_PLANNER_H

#include "main.h"
#include "Common/types.h"
#include "stdbool.h"
/**
 * @brief 任务状态机初始化（DRIVER_INIT 自动调用）
 */
// void mission_planner_init(void);

/**
 * @brief 任务状态机周期 tick（10ms 调用一次）
 *
 * 检测 FC 相位切换，驱动航点下发：
 * - WAITING_PATROL → 下发 Patrol 航点
 * - WAITING_RETURN → 下发 Return 航点
 * - LAND/DONE → 重置缓存
 *
 * 注：屏幕 UI 更新不在此层，由 touch_uart 层轮询 FC 相位自行处理。
 */
void mission_planner_tick(void);

/**
 * @brief 获取当前 FC 相位
 */
uint8_t mission_get_fc_phase(void);

/**
 * @brief 设置禁飞区（覆盖式）
 *
 * 同时置 zones_confirmed = true
 */
void mission_set_no_fly_zones(uint8_t count, struct Point_index_t* zones);

/**
 * @brief 重置禁飞区为空集 + zones_confirmed = false
 */
void mission_reset_no_fly_zones_v(void);

/**
 * @brief 计算 + 缓存双路线（内部，不做守卫）
 *
 * 一次计算 Patrol + Return，缓存压缩版（screen）和完整版（FC）。
 * 不发送 FC，不操作屏幕。
 */
void mission_request_route(void);

/**
 * @brief request_route 统一入口（含守卫）
 *
 * 守卫：zones_confirmed + 相位黑名单
 * @return true = 计算已执行，false = 被拦截
 */
bool mission_handle_request_route(void);

/**
 * @brief 获取巡逻路径（压缩网格坐标，供 screen 层发送预览）
 */
void mission_copy_patrol_screen_path_v(struct FlightPath* out);

/**
 * @brief 获取返航路径（压缩网格坐标，供 screen 层发送预览）
 */
void mission_copy_return_screen_path_v(struct FlightPath* out);



enum mission_send_phase_e
{
    MISSION_SEND_PHASE_IDLE_em,
    MISSION_SEND_PHASE_WAITTING_PATROL_em,
    MISSION_SEND_PHASE_PATROL_em,
    MISSION_SEND_PHASE_WAITTING_RETURN_em,
    MISSION_SEND_PHASE_RETURN_em,
};



#endif /* __MISSION_PLANNER_H */
