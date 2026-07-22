#ifndef __UPDATE_H
#define __UPDATE_H

#include "main.h"

/*
 * 传感器数据更新标志位
 *
 * Linux 内核风格：struct 定义在 .c（不透明），外部只通过 enum 索引访问。
 * 内部用数组代替 struct + switch，新增标志位只需在 .c 的字段表中加一行，
 * enum 自动增长，无需修改任何函数体。
 *
 * 生产者（ANO 协议回调）：update_flag_set_v(FIELD, 1)
 * 消费者（控制任务）：    if (update_flag_consume_uc(FIELD)) { ... }
 */

/* 标志位字段枚举 — 值即数组下标，新增字段追加到末尾即可 */
enum update_flag_field_e
{
    UPDATE_FLAG_REPORT_em,
    UPDATE_FLAG_REQUEST_PATROL_em,
    UPDATE_FLAG_REQUEST_RETURN_em,
};

enum update_state_e
{
    UPDATE_STATE_NO_DATA_em,
    UPDATE_STATE_HAS_DATA_em,
};
/* 通用"消费"标志位：读值后自动清零（防止重复处理） */
enum update_state_e update_flag_consume_uc(enum update_flag_field_e field_uc);

/* 通用"设置"标志位 */
void update_flag_set_v(enum update_flag_field_e field_uc);

/* 调试用：打印所有标志位状态（通过串口输出） */
// void update_flag_dump_v(void);

#endif /* __UPDATE_H */
