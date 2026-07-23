#include "mission_planner_base.h"
#include "route_planning_2024.h"
#include "driver_registry.h"

static struct mission_base_t s_mission_handle_st;



//24 年航线的申请点
const mission_ops_t route_2024_ops = {
    .request_route_b = all_the_route_planning_b,
    .can_route_b = can_route_planning_b,  
};


void mission_base_init_v(void)
{
    s_mission_handle_st.ops = &route_2024_ops;
}

DRIVER_INIT(mission_base_init_v);

bool mission_request_route_b(void)
{
    if (s_mission_handle_st.ops->request_route_b)
    {
        return s_mission_handle_st.ops->request_route_b();
    }
    
    return false;
}

bool mission_can_request_route_b(void)
{
    if (s_mission_handle_st.ops->can_route_b)
    {
        return s_mission_handle_st.ops->can_route_b();
    }
    return false;
}




