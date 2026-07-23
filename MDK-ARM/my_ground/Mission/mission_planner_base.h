#ifndef __MISSION_PLANNER_BASE_H
#define __MISSION_PLANNER_BASE_H


#include "main.h"

typedef struct mission_base_t mission_base_t;

typedef struct{
    bool (*request_route_b)(void);
    bool (*can_route_b)(void);

}mission_ops_t;


//绑定的操作函数
struct mission_base_t {
    mission_ops_t* ops;
};

bool mission_request_route_b(void);
bool mission_can_request_route_b(void);


#endif
