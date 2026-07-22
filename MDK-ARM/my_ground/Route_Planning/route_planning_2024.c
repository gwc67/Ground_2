#include "route_planning_2024.h"
#include "driver_registry.h"
#include "point_3d.h"
#include "map/map.h"
#include "math.h"

//最好是封装一个映射函数，能够将世界坐标，映射到具体的A，B系里面
//建议航线规划使用世界坐标，而航点显示，简单使用普通映射系即可


struct shelf_param_t {
    int16_t origin_x_s;     //货架X 坐标
    int16_t y_first_col_s;  // 第一列 Y 坐标
    uint8_t num_cols_uc;    //列数
    int16_t col_step_s;     //列间距
};



//根据点的坐标和yaw角，判断是哪个货架
const struct shelf_param_t c_shelf_param_pst[] = {
    /* A */ {.origin_x_s =   0, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    /* B */ {.origin_x_s = 150, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    /* C */ {.origin_x_s = 200, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    /* D */ {.origin_x_s = 350, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
};

#define SHELF_NUM sizeof(c_shelf_param_pst)/sizeof(c_shelf_param_pst[0])

#define AISLE_Y_1   -25         //U形转换避障的坐标 偏下面的
#define AISLE_Y_2   225         //U形转换避障的坐标，偏上的坐标

#define YAW_FONT        0
#define YAW_REAR        180



//先确定target_st 所在的货架，再进行规划算法即可



struct Point_3D_t target_st = {.x_s = 200,.y_s = 75,.z_s = 140,.yaw_s = 0}; 
struct Point_3D_t return_st = {.x_s = 350,.y_s = 250,.z_s = 140,.yaw_s = 0};  //目标的yaw_s 和 





static void s_identify_shelf_face(const struct Point_3D_t* target_pst,struct shelf_param_t* matched_face_pst )
{

    if(target_pst->yaw_s == YAW_FONT)  
    {
        if (target_pst->x_s > 150)
        {
            *matched_face_pst = c_shelf_param_pst[2]; //C面
        }
        else
        {
            *matched_face_pst = c_shelf_param_pst[0]; //A面
        }
        
    }
    else if (target_pst->yaw_s == YAW_REAR)
    {
        if (target_pst->x_s > 200)
        {
            *matched_face_pst = c_shelf_param_pst[3];//B面
        }
        else
        {
            *matched_face_pst = c_shelf_param_pst[1]; //D面
        }
    }
}



void route_generate_patrol(const struct Point_3D_t* target_pst)
{


    struct shelf_param_t shelf_target_st;

    s_identify_shelf_face_b(target_pst,&shelf_target_st);     //获得此时的货架

    struct Point_3D_t target_cur_st = {.yaw_s = target_pst->yaw_s,.z_s = target_pst->z_s};

    if (&shelf_target_st == c_shelf_param_pst)                //如果是A面就直接进行飞行就行
    {
        target_cur_st.x_s = target_pst->x_s;
        target_cur_st.y_s = target_pst->y_s;
        point_3d_add_b(&target_st);
    }


    
        

    
    


}









