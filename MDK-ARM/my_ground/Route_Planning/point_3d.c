#include "point_3d.h"
#include "types.h"
#include "ring_buffer.h"
#include "driver_registry.h"

#define TARGET_QUEUE_CAPACITY 64

static stRingBufTdf s_target_fifo_st;
static uint8_t s_target_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t)];

static void s_point_3d_init_v(void)
{
    vRingBufItemInit(&s_target_fifo_st,
                     TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t),
                     sizeof(struct Point_3D_t),
                     s_target_buf);
}
DRIVER_INIT(s_point_3d_init_v);

bool point_3d_is_empty_b(void)
{
    return ucRingBufIsEmpty(&s_target_fifo_st);
}

void point_3d_clear_b(void)
{
    ucRingBufClear(&s_target_fifo_st);
}

bool point_3d_add_b(const struct Point_3D_t *pst)
{
    return (ucRingBufWriteItem(&s_target_fifo_st, pst) == 0);
}

void point_3d_take_t(struct Point_3D_t* point_pst)
{
    ucRingBufReadItem(&s_target_fifo_st,&point_pst);
}


