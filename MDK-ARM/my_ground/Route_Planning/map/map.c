#include "map.h"
#include "types.h"
#include "ring_buffer.h"
#include "point_2d.h"
#include "point_3d.h"

#define MAP_WIDTH   50     //对标x 坐标
#define MAP_LENGTH  50     //对标y 坐标

#define MAP_X_NUMS  8      //X索引
#define MAP_Y_NUMS  6       //Y索引


//获得一个世界坐标后，应该存起来

void map_set_v(struct Point_3D_t* in) 
{
    struct Point_2D_t point_2d_st = {0};

    point_2d_st.x_c = (in->x_s - MAP_WIDTH)/ MAP_WIDTH;             //支持负坐标
    point_2d_st.y_c = (in->y_s - MAP_LENGTH) / MAP_LENGTH;          //支持负坐标

    point_2d_add_b(&point_2d_st);

}








