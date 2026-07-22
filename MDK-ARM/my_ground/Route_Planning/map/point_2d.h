#ifndef __POINT_INDEX_H
#define __POINT_INDEX_H

#include "main.h"

struct Point_2D_t{
    int8_t x_c;
    int8_t y_c;
};


bool point_2d_is_empty_b(void);

void point_2d_clear_b(void);

bool point_2d_add_b(const struct Point_2D_t *pst);

void point_2d_take_t(struct Point_2D_t* point_pst);

#endif
