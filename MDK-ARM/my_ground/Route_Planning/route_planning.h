#ifndef __ROUTE_PLANNING_H
#define __ROUTE_PLANNING_H

#include "Common/types.h"
#include <stdbool.h>

/**
 * @brief 网格坐标 → 世界坐标转换
 * @param grid_x 网格 X（1-9，AB 系列号）
 * @param grid_y 网格 Y（1-7，AB 系行号）
 * @param world_x 输出：世界 X（cm）
 * @param world_y 输出：世界 Y（cm）
 *
 * 转换公式（从 FC Route_Planning.c:522-533）：
 *   world_x = (grid_y - 1) * 50
 *   world_y = (9 - grid_x) * 50
 */
void grid_to_world(uint8_t grid_x, uint8_t grid_y, int16_t *world_x, int16_t *world_y);

/**
 * @brief 压缩航线（移除共线中间点）
 * @param original 原始航线
 * @param compressed 输出：压缩后航线
 */
void compress_path(struct FlightPath *original, struct FlightPath *compressed);

/**
 * @brief 规划巡逻路径
 * @param path 输出：路径点序列（网格坐标）
 * @param grid 障碍物地图（0=空，1=障碍）
 * @param start_x 起点 X（网格坐标）
 * @param start_y 起点 Y（网格坐标）
 * @param horizontal true=水平扫描，false=垂直扫描
 * @return 路径点数量
 */
uint8_t plan_path(struct Point_index_t *path, uint8_t grid[WIDTH][HEIGHT],
                  uint8_t start_x, uint8_t start_y, bool horizontal);

/**
 * @brief 规划返航路径
 * @param path 输入/输出：追加返航点到现有路径
 * @param index 输入/输出：当前路径长度
 * @param grid 障碍物地图
 * @param start_index 返航起点在 path 中的索引
 * @param ret_fp 输出：返航航线（网格坐标，1-indexed）
 */
void plan_return_path(struct Point_index_t *path, uint8_t *index, uint8_t grid[WIDTH][HEIGHT],
                      uint8_t start_index, struct FlightPath *ret_fp);

/**
 * @brief 判断障碍物是否呈水平分布
 * @param grid 障碍物地图
 * @return true=水平扫描模式，false=垂直扫描模式
 */
bool is_block_horizontal(uint8_t grid[WIDTH][HEIGHT]);

/**
 * @brief 完整任务规划（巡逻 + 返航）
 * @param no_fly_zones 禁飞区（3 个网格点，1-indexed）
 * @param start_x 起点 X（网格坐标）
 * @param start_y 起点 Y（网格坐标）
 * @param patrol_pts 输出：巡逻航点（世界坐标）
 * @param patrol_cnt 输出：巡逻航点数量
 * @param return_pts 输出：返航航点（世界坐标）
 * @param return_cnt 输出：返航航点数量
 *
 * 流程：
 * 1. 构建障碍物地图（从 no_fly_zones）
 * 2. 判断扫描方向
 * 3. 规划巡逻路径（网格坐标）
 * 4. 压缩路径
 * 5. 规划返航路径（网格坐标）
 * 6. 压缩返航路径
 * 7. 坐标转换（网格 → 世界）
 */
void plan_mission(struct Point_index_t no_fly_zones[3],
                  uint8_t start_x, uint8_t start_y,
                  struct Point_t *patrol_pts, uint8_t *patrol_cnt,
                  struct Point_t *return_pts, uint8_t *return_cnt);

#endif /* __ROUTE_PLANNING_H */
