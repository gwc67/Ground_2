#include "point_2d.h"
#include "ring_buffer.h"
#include "driver_registry.h"



// static stRingBufTdf s_target_fifo_st;
// static uint8_t s_target_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_2D_t)];
// 加一个添加一个，还是先一口气添加完再压缩一次，还是额外写一个接口用于压缩
static struct Point_map_t s_point_map_patrol_st = {0};
static struct Point_map_t s_point_map_return_st = {0};

struct point_2d_base_t* g_patrol_point_2d_pst;
struct point_2d_base_t* g_return_point_2d_pst;


static struct point_2d_base_t* s_point_2d_base_init(struct Point_map_t* point_map_pst,const char* name)
{
    point_map_pst->base.name = name;
    return &point_map_pst->base;
}

void point_map_init(void)
{
    g_patrol_point_2d_pst =  s_point_2d_base_init(&s_point_map_patrol_st,"patrol_2d");
    g_return_point_2d_pst =  s_point_2d_base_init(&s_point_map_return_st,"return_2d");
}
DRIVER_INIT(point_map_init);


void point_2d_clear_b(struct point_2d_base_t* base)
{
    struct Point_map_t* me = container_of(base,struct Point_map_t,base);
    me->count_uc = 0;

}

bool point_2d_add_b(struct point_2d_base_t* base ,const struct Point_2D_t *pst)
{
    struct Point_map_t* me = container_of(base,struct Point_map_t,base);
    if (me->count_uc < POINT_MAP_LENGTH)
    {
        me->point_mat_pst[me->count_uc++] = *pst;
        return true;
    }
    else
    {
        return false;
    }
}

void  point_map_take_v(struct point_2d_base_t* base, struct Point_map_t* point_map_pst)
{
    struct Point_map_t* me = container_of(base,struct Point_map_t,base);
    *point_map_pst = *me ;
}



//将局部变量压缩到全局串口屏接受里面
void route_compress_v(struct Point_map_t* raw_pst,struct point_2d_base_t* base)
{
    //如果没有压缩路径

    struct Point_map_t* me = container_of(base,struct Point_map_t,base);

    if (raw_pst->count_uc == 0)
    {
        me->count_uc = 0;
        return;
    }
    
    me->count_uc = 1;
    me->point_mat_pst[0] = raw_pst->point_mat_pst[0];          //赋值第一个点

    if (raw_pst->count_uc == 1)                                     //raw有一个点，那就只压缩一个点
    {
        return;
    }

    int8_t dx_c = (int8_t)raw_pst->point_mat_pst[1].x_c - (int8_t)raw_pst->point_mat_pst[0].x_c;
    int8_t dy_c = (int8_t)raw_pst->point_mat_pst[1].y_c - (int8_t)raw_pst->point_mat_pst[0].y_c;
    int8_t last_dx_c = dx_c;
    int8_t last_dy_c = dy_c;

    for (uint8_t i = 2; i < raw_pst->count_uc; i++)
    {
        dx_c = (int8_t)raw_pst->point_mat_pst[i].x_c - (int8_t)raw_pst->point_mat_pst[i - 1].x_c;
        dy_c = (int8_t)raw_pst->point_mat_pst[i].y_c - (int8_t)raw_pst->point_mat_pst[i - 1].y_c;
        
        if (dx_c != last_dx_c || dy_c != last_dy_c)
        {
            me->point_mat_pst[me->count_uc++] = raw_pst->point_mat_pst[i - 1];   

            last_dx_c = dx_c;
            last_dy_c = dy_c;
        }
        
    }
    
    //最后一个点
    me->point_mat_pst[me->count_uc++]  = raw_pst->point_mat_pst[raw_pst->count_uc - 1];
        
}



