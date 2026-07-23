#include "point_2d.h"
#include "ring_buffer.h"
#include "driver_registry.h"



// static stRingBufTdf s_target_fifo_st;
// static uint8_t s_target_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_2D_t)];

static struct Point_map_t s_point_map_st = {0};


void point_2d_clear_b(void)
{
    s_point_map_st.count_uc = 0;
}

bool point_2d_add_b(const struct Point_2D_t *pst)
{
    if (s_point_map_st.count_uc < POINT_MAP_LENGTH)
    {
        s_point_map_st.point_mat_pst[s_point_map_st.count_uc++] = *pst;
        return true;
    }
    else
    {
        return false;
    }
}

void    point_map_take_t(struct Point_map_t* point_map_pst)
{
     *point_map_pst = s_point_map_st ;
}

