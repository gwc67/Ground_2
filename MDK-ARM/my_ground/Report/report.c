#include "report.h"
#include "string.h"
#include "driver_registry.h"
#include "stdbool.h"
#define DELIVERY_MAX_NUM   24                                     //货物的数量

//这东西我认为没有必要上环形缓冲区，它要保证一直都在

static struct delivery_t s_all_the_delivery_pst[DELIVERY_MAX_NUM] = {
    [0] = {.type_uc = 1,.x_s = 150,.y_s = 75,.z_s = 140,.yaw_s = 180},  //B面
    [1] = {.type_uc = 2,.x_s = 300,.y_s = 75,.z_s = 140,.yaw_s = 180},  //D 面 
};
static struct delivery_t s_delivery_special_st;
static uint8_t s_delivery_index_uc;                              //货物索引号，每添加一个就自增，考虑接受端，想要查询货物可以怎么做呢？应该根据货物的id号进行查询，也就是id_uc，这个表面标签,这个变量名只负责自增添加；


void delivery_init_v(void)
{
    memset(s_all_the_delivery_pst, 0, sizeof(s_all_the_delivery_pst));
    s_delivery_index_uc = 0;
}

DRIVER_INIT(delivery_init_v);

//这个可以进行重复添加，可以啊


/// @brief -1 超过索引 正常是返回此时的已有索引
/// @param p_new 
/// @return 
int8_t delivery_add_b(struct delivery_t *p_new)
{
    // 边界保护
    if (s_delivery_index_uc >= DELIVERY_MAX_NUM)
    {
        return -1;
    }
    
    // 去重：同一个 type 不重复添加
    for (uint8_t i = 0; i < s_delivery_index_uc; i++)
    {
        if (s_all_the_delivery_pst[i].type_uc == p_new->type_uc)
        {
            return (int8_t)i;  // 已存在，返回它所存在的索引
        }
    }
    
    // 写入新槽位
    s_all_the_delivery_pst[s_delivery_index_uc] = *p_new;
    
    int8_t idx = (int8_t)s_delivery_index_uc;
    s_delivery_index_uc++;
    
    return idx;
}

bool delivery_find_by_type_b(uint8_t type_uc, struct delivery_t *out)
{
    for (uint8_t i = 0; i < s_delivery_index_uc; i++)
    {
        if (s_all_the_delivery_pst[i].type_uc == type_uc)
        {
            *out = s_all_the_delivery_pst[i];
            return true;                                    //找到即返回就可以
        }
    }
    return false;
}


bool delivery_copy_by_position_b(uint8_t position_uc, struct delivery_t *out)
{
    for (uint8_t i = 0; i < s_delivery_index_uc; i++)
    {
        if (s_all_the_delivery_pst[i].position_uc == position_uc)
        {
            *out = s_all_the_delivery_pst[i];
            return true;
        }
    }
    return false;
}


bool delivery_copy_by_index_b(uint8_t index_uc, struct delivery_t *out)
{
    if (index_uc >= s_delivery_index_uc)
    {
        return false;
    }
    *out = s_all_the_delivery_pst[index_uc];
    return true;
}

uint8_t delivery_get_cur_index(void)
{
    return s_delivery_index_uc;
}

//设置特定的货物的种类，同时待会飞控还要显示特殊的货物类型，根据已知的货物类型进行推导
void delivery_set_special(struct delivery_t* in)
{
    s_delivery_special_st = *in;
}

void delivery_copy_special(struct delivery_t* out)
{
    *out = s_delivery_special_st;
}

uint8_t delivery_get_special_type_uc(void)
{
    return s_delivery_special_st.type_uc;
}
