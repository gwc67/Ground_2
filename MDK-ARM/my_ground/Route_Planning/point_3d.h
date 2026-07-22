#ifndef __POINT_3D_H
#define __POINT_3D_H

#include "main.h"

struct Point_3D_t{
    int16_t x_s;
    int16_t y_s;
    int16_t z_s;
    int16_t yaw_s;
};


bool point_3d_is_empty_b(void);

void point_3d_clear_b(void);

bool point_3d_add_b(const struct Point_3D_t *pst);

void point_3d_take_t(struct Point_3D_t* point_pst);



#endif
