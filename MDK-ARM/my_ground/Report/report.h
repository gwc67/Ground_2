#ifndef __REPORT_H
#define __REPORT_H

#include "main.h"

struct delivery_t {
    int16_t x_s;            //世界坐标X
    int16_t y_s;            //世界坐标Y
    int16_t z_s;
    uint8_t type_uc;        //货物编号 1~24  会给无人机看一张货物的编号，也就是这个值，无人机报送识别的货物编号给地面站显示，地面站显示规划的定点盘点图
    uint8_t position_uc;    //坐标信息 A1 ~ A6 B1 ~ B6 C1 ~ C6 D1 ~ D6
};


int8_t delivery_add_b(struct delivery_t *p_new);

bool delivery_find_by_type_b(uint8_t type_uc, struct delivery_t *out);

bool delivery_copy_by_position_b(uint8_t position_uc, struct delivery_t *out);

bool delivery_copy_by_index_b(uint8_t index_uc, struct delivery_t *out);

uint8_t delivery_get_cur_index(void);

#endif
