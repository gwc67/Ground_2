#ifndef __POINT_INDEX_H
#define __POINT_INDEX_H

#include "main.h"

#define POINT_MAP_LENGTH 64


//这里没有上缓冲区，方便串口屏显示航线，如果被清除了话
struct Point_2D_t{
    int8_t x_c;
    int8_t y_c;
};


struct Point_map_t
{
    uint8_t count_uc;
    struct Point_2D_t  point_mat_pst[POINT_MAP_LENGTH]; 
};




void point_2d_clear_b(void);

bool point_2d_patrol_add_b(const struct Point_2D_t *pst);

void point_2d_patrol_take_v(struct Point_map_t* point_map_pst); 

bool point_2d_return_add_b(const struct Point_2D_t *pst);

void point_2d_return_take_v(struct Point_map_t* point_map_pst);


#endif
