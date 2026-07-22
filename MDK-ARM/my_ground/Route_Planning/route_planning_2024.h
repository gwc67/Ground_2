#ifndef __ROUTE_PLANNING_2024_H
#define __ROUTE_PLANNING_2024_H


#include "main.h"
#include "point_3d.h"
void route_generate_patrol(const struct Point_3D_t* target_pst); 
void route_generate_return(const struct Point_3D_t* target_pst);

#endif
