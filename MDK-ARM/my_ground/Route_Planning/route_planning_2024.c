#include "route_planning_2024.h"
#include "driver_registry.h"
#include "point_3d.h"
#include "map/map.h"
#include "math.h"

//最好是封装一个映射函数，能够将世界坐标，映射到具体的A，B系里面
//建议航线规划使用世界坐标，而航点显示，简单使用普通映射系即可


struct Point_3D_t target_st = {.x_s = 200,.y_s = 75,.z_s = 140,.yaw_s = 0}; 
struct Point_3D_t return_st = {.x_s = 350,.y_s = 250,.z_s = 140,.yaw_s = 0};  //目标的yaw_s 和 







#define AISLE_Y_1   -25         //U形转换避障的坐标 偏下面的
#define AISLE_Y_2   275         //U形转换避障的坐标，偏上的坐标

#define YAW_FONT        0
#define YAW_REAR        180


// 建议：给货架增加ID字段，避免用内存比较
enum shelf_id_e { SHELF_A = 0, SHELF_B, SHELF_C, SHELF_D } ;

struct shelf_param_t {
    enum shelf_id_e id;          // ← 新增标识
    int16_t origin_x_s;
    int16_t y_first_col_s;
    uint8_t num_cols_uc;
    int16_t col_step_s;
};

const struct shelf_param_t c_shelf_param_pst[] = {
    [SHELF_A] = {.id = SHELF_A, .origin_x_s =   0, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    [SHELF_B] = {.id = SHELF_B, .origin_x_s = 150, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    [SHELF_C] = {.id = SHELF_C, .origin_x_s = 200, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
    [SHELF_D] = {.id = SHELF_D, .origin_x_s = 350, .y_first_col_s = 75, .num_cols_uc = 3, .col_step_s = 50},
};

#define SHELF_NUM sizeof(c_shelf_param_pst)/sizeof(c_shelf_param_pst[0])

// 修正：统一函数名，通过ID识别货架
static void s_identify_shelf_face(const struct Point_3D_t* target_pst, 
                                   const struct shelf_param_t** matched_ppst)
{
    if (target_pst->yaw_s == YAW_FONT) {
        *matched_ppst = (target_pst->x_s > 100) 
                        ? &c_shelf_param_pst[SHELF_C] 
                        : &c_shelf_param_pst[SHELF_A];
    } else if (target_pst->yaw_s == YAW_REAR) {
        // 注意：x>200 对应D面(origin=350)，x<=250 对应B面(origin=150)
        *matched_ppst = (target_pst->x_s > 250) 
                        ? &c_shelf_param_pst[SHELF_D] 
                        : &c_shelf_param_pst[SHELF_B];
    }
}

void route_generate_patrol(const struct Point_3D_t* target_pst)
{
    const struct shelf_param_t* shelf_pst = NULL;
    s_identify_shelf_face(target_pst, &shelf_pst);  
    
    if (shelf_pst == NULL) return;  

    struct Point_3D_t wp = {.yaw_s = target_pst->yaw_s, .z_s = target_pst->z_s};

    if (wp.yaw_s == YAW_REAR)
    {
        point_3d_add_b(&wp);            //先转到指定yaw角
    }
    

    if (shelf_pst->id == SHELF_A) {
        // A面直达
        wp.x_s = target_pst->x_s;
        wp.y_s = target_pst->y_s;
        point_3d_add_b(&wp);
    } else {
        wp.y_s = AISLE_Y_1;
        point_3d_add_b(&wp);

        wp.x_s = target_pst->x_s;  // 保持当前X进入巷道
        point_3d_add_b(&wp);

        point_3d_add_b(target_pst);  // 最终目标点
    }

    // 返程航点（非D面时才添加）
    if (shelf_pst->id != SHELF_D) {
        wp.y_s = AISLE_Y_2;
        wp.x_s = return_st.x_s;   
        point_3d_add_b(&wp);
    }
}








