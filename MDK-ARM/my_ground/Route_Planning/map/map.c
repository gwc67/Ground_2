#include "map.h"
#include "types.h"
#include "ring_buffer.h"
#include "point_2d.h"
#include "point_3d.h"

#define MAP_WIDTH   50     //对标x 坐标
#define MAP_LENGTH  50     //对标y 坐标

#define MAP_X_NUMS  8      //X索引
#define MAP_Y_NUMS  6       //Y索引

#define GRID_STEP       50
#define GRID_OFFSET     25      // 半个步长，实现中心对齐



//获得一个世界坐标后，应该存起来

void map_set_v(struct Point_3D_t* in) 
{
    struct Point_2D_t point_2d_st = {0};

    int16_t x_shifted =   in->x_s  + GRID_OFFSET;
    int16_t y_shifted =   in->y_s  + GRID_OFFSET;

    point_2d_st.x_c = (int8_t)(x_shifted >= 0 
                          ? x_shifted / GRID_STEP 
                          : (x_shifted - GRID_STEP + 1) / GRID_STEP);

    point_2d_st.y_c = (int8_t)(y_shifted >= 0 
                          ? y_shifted / GRID_STEP 
                          : (y_shifted - GRID_STEP + 1) / GRID_STEP);


    point_2d_patrol_add_b(&point_2d_st);

}








