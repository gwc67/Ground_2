#ifndef __COMMON_TYPES_H
#define __COMMON_TYPES_H

#include "main.h"

/* ============== 共享数据类型（无 typedef，Linux 风格） ============== */

#define WIDTH    9
#define HEIGHT   7
#define MAX_PATH 200

/* 网格坐标（0-indexed，AB 系） */
struct Point_index_t {
    uint8_t x;
    uint8_t y;
};
/* 航线（网格坐标序列） */
// 航线使用网格坐标进行计算
struct FlightPath {
    uint8_t count;
    struct Point_index_t points[MAX_PATH];
};
/* 世界坐标（cm，飞控期望格式） */
struct Point_t {
    int16_t x;
    int16_t y;
};
#endif /* __COMMON_TYPES_H */
