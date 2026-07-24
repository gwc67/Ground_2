#include "point_3d.h"
#include "types.h"
#include "ring_buffer.h"
#include "driver_registry.h"

#define TARGET_QUEUE_CAPACITY 64


struct point_fifo_t
{
    struct point_3d_base base;
    stRingBufTdf fifo_st;
};



static struct point_fifo_t s_patrol_fifo_st;
static uint8_t s_patrol_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t)];

static struct point_fifo_t s_return_fifo_st;
static uint8_t s_return_buf[TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t)];

struct point_3d_base* g_patrol_point_3d_pst;
struct point_3d_base* g_return_point_3d_pst;

static struct point_3d_base* s_point_fifo_base_init(struct point_fifo_t* fifo_pst,const char* name)
{
    fifo_pst->base.name = name;
    return &(fifo_pst->base);
}


static void s_point_3d_init_v(void)
{
    vRingBufItemInit(&s_patrol_fifo_st.fifo_st,
                     TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t),
                     sizeof(struct Point_3D_t),
                     s_patrol_buf);
    g_patrol_point_3d_pst = s_point_fifo_base_init(&s_patrol_fifo_st,"patrol");

    vRingBufItemInit(&s_return_fifo_st.fifo_st,
                     TARGET_QUEUE_CAPACITY * sizeof(struct Point_3D_t),
                     sizeof(struct Point_3D_t),
                     s_return_buf);
    g_return_point_3d_pst = s_point_fifo_base_init(&s_return_fifo_st,"return");
    
}
DRIVER_INIT(s_point_3d_init_v);

bool point_3d_is_empty_b(struct point_3d_base* base)
{
    struct point_fifo_t* me = container_of(base,struct point_fifo_t,base);
    return ucRingBufIsEmpty(&(me->fifo_st));
}

void point_3d_clear_b(struct point_3d_base* base)
{
    struct point_fifo_t* me = container_of(base,struct point_fifo_t,base);
    ucRingBufClear(&(me->fifo_st));
}

bool point_3d_add_b(struct point_3d_base* base,const struct Point_3D_t *point_pst)
{
    struct point_fifo_t* me = container_of(base,struct point_fifo_t,base);
    return (ucRingBufWriteItem(&(me->fifo_st), point_pst) == 0);
}

uint8_t point_3d_take_uc(struct point_3d_base* base,struct Point_3D_t* point_pst)
{
    struct point_fifo_t* me = container_of(base,struct point_fifo_t,base);
    return ucRingBufReadItem(&(me->fifo_st),point_pst);
}


