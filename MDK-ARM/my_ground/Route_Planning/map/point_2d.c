#include "point_2d.h"
#include "ring_buffer.h"
#include "driver_registry.h"



// static stRingBufTdf s_target_fifo_st;
// static uint8_t s_target_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_2D_t)];

static struct Point_map_t s_point_map_patrol_st = {0};
static struct Point_map_t s_point_map_return_st = {0};


void point_2d_clear_b(void)
{
    s_point_map_patrol_st.count_uc = 0;
    s_point_map_return_st.count_uc = 0;
}

bool point_2d_patrol_add_b(const struct Point_2D_t *pst)
{
    if (s_point_map_patrol_st.count_uc < POINT_MAP_LENGTH)
    {
        s_point_map_patrol_st.point_mat_pst[s_point_map_patrol_st.count_uc++] = *pst;
        return true;
    }
    else
    {
        return false;
    }
}

void  point_2d_patrol_take_v(struct Point_map_t* point_map_pst)
{
     *point_map_pst = s_point_map_patrol_st ;
}


bool point_2d_return_add_b(const struct Point_2D_t *pst)
{
    if (s_point_map_return_st.count_uc < POINT_MAP_LENGTH)
    {
        s_point_map_return_st.point_mat_pst[s_point_map_return_st.count_uc++] = *pst;
        return true;
    }
    else
    {
        return false;
    }
}

void point_2d_return_take_v(struct Point_map_t* point_map_pst)
{
     *point_map_pst = s_point_map_return_st ;
}

