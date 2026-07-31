#ifndef __ROUTE_PLANNING_2025_H
#define __ROUTE_PLANNING_2025_H

#include "main.h"

enum block_loc_e{
    BLOCK_LOC_NO_REGULAR_em,        //没有规则的BLOCK_LOC_NO

    BLOCK_LOC_HORIZONTAL_em,        //水平
    BLOCK_LOC_HORIZONTAL_UP_em,
    BLOCK_LOC_HORIZONTAL_DOWN_em,
    BLOCK_LOC_HORIZONTAL_RIGHT_em,
    BLOCK_LOC_HORIZONTAL_LEFT_em,
    
    BLOCK_LOC_VERTICAL_em,                  //垂直          
    BLOCK_LOC_VERTICAL_UP_em,
    BLOCK_LOC_VERTICAL_DOWN_em,
    // BLOCK_LOC_VERTICAL_RIGHT_em,
    // BLOCK_LOC_VERTICAL_LEFT_em,
};

enum probe_point_e{
    PROBE_POINT_IDLE_em,
    PROBE_POINT_BEGIN_PROBE_em,
    PROBE_POINT_WHICH_MOVE_em,
    PROBE_POINT_CAN_MOVE_UP_em,             //都是相对参考系，不是绝对参考系
    PROBE_POINT_CAN_MOVE_DOWN_em,
    PROBE_POINT_CAN_MOVE_ON_em,
    PROBE_POINT_CAN_MOVE_BACK_em,
    PROBE_POINT_CAN_NOT_MOVE_UP_em,             //都是相对参考系，不是绝对参考系
    PROBE_POINT_CAN_NOT_MOVE_DOWN_em,
    PROBE_POINT_CAN_NOT_MOVE_ON_em,
    PROBE_POINT_CAN_NOT_MOVE_BACK_em,
};

void plan_path_v(void);
void plan_path_new(void);



#endif

