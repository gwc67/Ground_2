#ifndef __POINT_3D_H
#define __POINT_3D_H

#include "main.h"

struct Point_3D_t{
    int16_t x_s;
    int16_t y_s;
    int16_t z_s;
    int16_t yaw_s;
};

struct point_3d_base{
    const char* name;
};


extern struct point_3d_base* g_partrol_point_3d_pst;
extern struct point_3d_base* g_return_point_3d_pst;

bool point_3d_is_empty_b(struct point_3d_base* base);


void point_3d_clear_b(struct point_3d_base* base);

bool point_3d_add_b(struct point_3d_base* base,const struct Point_3D_t *point_pst);

void point_3d_take_t(struct point_3d_base* base,struct Point_3D_t* point_pst);



#endif
