#ifndef __POINT_INDEX_H
#define __POINT_INDEX_H

#include "main.h"

#define POINT_MAP_LENGTH 64


//这里没有上缓冲区，方便串口屏显示航线，如果被清除了话
struct Point_2D_t{
    int8_t x_c;
    int8_t y_c;
};

struct point_2d_base_t
{
    const char* name;
};

struct Point_map_t
{
    struct point_2d_base_t base;
    uint8_t count_uc;
    struct Point_2D_t  point_mat_pst[POINT_MAP_LENGTH]; 
};


extern struct point_2d_base_t* g_patrol_point_2d_pst;
extern struct point_2d_base_t* g_return_point_2d_pst;

void point_2d_clear_b(struct point_2d_base_t* base);

bool point_2d_add_b(struct point_2d_base_t* base, const struct Point_2D_t *pst);

void  point_map_take_v(struct point_2d_base_t* base, struct Point_map_t* point_map_pst);

void route_compress_v(struct Point_map_t* raw_pst,struct point_2d_base_t* base);


#endif
