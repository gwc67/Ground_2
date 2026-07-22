#ifndef __MAP_H
#define __MAP_H

//这个是给那些进行世界坐标，转换成网格坐标进行使用的，具体画的地图将由对应的结构体进行二次修改，否则默认使用，
//本质是为了给航点进行服务的，航点计算使用世界坐标，之后转换成世界坐标进行控制
//那不是应该使用point_index才对？



#include "main.h"
#include "point_3d.h"
void map_set_v(struct Point_3D_t* in) ;


#endif

