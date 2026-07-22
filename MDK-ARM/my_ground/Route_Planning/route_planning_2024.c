#include "route_planning_2024.h"
#include "driver_registry.h"
#include "point_3d.h"
#include "map/map.h"
#include "math.h"

//最好是封装一个映射函数，能够将世界坐标，映射到具体的A，B系里面
//建议航线规划使用世界坐标，而航点显示，简单使用普通映射系即可


struct Point_3D_t return_st = {.x_s = 350,.y_s = 250,.z_s = 140,.yaw_s = 0};  //目标的yaw_s 和 



enum shelf_e{
    SHELF_A,
    SHELF_B,
    SHELF_C,
    SHELF_D,
};



#define AISLE_Y_1   -25         //U形转换避障的坐标 偏下面的
#define AISLE_Y_2   275         //U形转换避障的坐标，偏上的坐标

#define YAW_FONT        0
#define YAW_REAR        180

#define SHELF_NUM sizeof(c_shelf_param_pst)/sizeof(c_shelf_param_pst[0])

//此时的地面站，应该要显示所有得航线才对
// 修正：统一函数名，通过ID识别货架
static enum shelf_e s_identify_shelf_face_em(const struct Point_3D_t* target_pst)
{
    enum shelf_e id_em = SHELF_A;
    if (target_pst->yaw_s == YAW_FONT) {
        id_em = (target_pst->x_s > 100) 
                        ? SHELF_C 
                        : SHELF_A;
    } else if (target_pst->yaw_s == YAW_REAR) {
        // 注意：x>200 对应D面(origin=350)，x<=250 对应B面(origin=150)
        id_em = (target_pst->x_s > 250) 
                        ? SHELF_D
                        : SHELF_B;
    }
    return id_em;
}


static struct Point_3D_t target_cur_st = {0};
void route_generate_patrol(const struct Point_3D_t* target_pst)
{

    enum shelf_e shelf_em =  s_identify_shelf_face_em(target_pst);  
    

    target_cur_st.yaw_s = target_pst->yaw_s;
    target_cur_st.z_s = target_pst->z_s;

    if (target_cur_st.yaw_s == YAW_REAR)
    {
        point_3d_add_b(g_partrol_point_3d_pst,&target_cur_st);            //先转到指定yaw角
        map_set_v(&target_cur_st);
    }
    

    if (shelf_em == SHELF_A) {
        // A面直达
        target_cur_st.x_s = target_pst->x_s;
        target_cur_st.y_s = target_pst->y_s;
        point_3d_add_b(g_partrol_point_3d_pst,&target_cur_st);
        map_set_v(&target_cur_st);
    } else {
        target_cur_st.y_s = AISLE_Y_1;
        point_3d_add_b(g_partrol_point_3d_pst,&target_cur_st);
        map_set_v(&target_cur_st);


        target_cur_st.x_s = target_pst->x_s;  // 保持当前X进入巷道
        point_3d_add_b(g_partrol_point_3d_pst,&target_cur_st);
        map_set_v(&target_cur_st);

        point_3d_add_b(g_partrol_point_3d_pst,target_pst);  // 最终目标点
        map_set_v(&target_cur_st);

    }
}


//在这里补一个航点就能做到45°降落，当然得看y与z轴得大小，关系否则得提前降落了，现在先不管
void route_generate_return(const struct Point_3D_t* target_pst)
{
    // 返程航点（非D面时才添加）

    enum shelf_e shelf_em =  s_identify_shelf_face_em(target_pst);  

    
    if (shelf_em != SHELF_D) {
        target_cur_st.y_s = AISLE_Y_2;

        point_3d_add_b(g_return_point_3d_pst,&target_cur_st);
        map_set_v(&target_cur_st);
        
        target_cur_st.x_s = return_st.x_s;   

        point_3d_add_b(g_return_point_3d_pst,&target_cur_st);
        map_set_v(&target_cur_st);

    }
    
    //最后添加一个返航终点即可                  //45°降落，我想一下，没关系，我的缓冲区可以获取最后一个长度判断是否是以及最后一个航点，可以实现
    target_cur_st.x_s = return_st.x_s;
    target_cur_st.y_s = return_st.y_s;
    point_3d_add_b(g_return_point_3d_pst,&target_cur_st);
    map_set_v(&target_cur_st);      

}








