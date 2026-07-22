#ifndef __ANO_PROTOCOL_H
#define __ANO_PROTOCOL_H

#include "main.h"

#define FUN_NUM_LEN 256

typedef struct
{
    u8 D_Addr;
    u8 WTS;
    u16 fre_ms;
    u16 time_cnt_ms;
} _dt_frame_st;

typedef struct
{
    u8 CID;
    u8 CMD[10];
} _cmd_st;

typedef struct
{
    u8 ID;
    u8 SC;
    u8 AC;
} _ck_st;

typedef struct
{
    u16 par_id;
    s32 par_val;
} _par_st;

typedef struct
{
    _dt_frame_st fun[FUN_NUM_LEN];
    u8 wait_ck;
    u8 ucRepeat_cnt;
    u8 ucTime_dly;
    _cmd_st cmd_send;
    _ck_st ck_send;
    _ck_st ck_back;
    _par_st par_data;
    /* CMD 队列（1 槽）：wait_ck 被占用时缓存待发送的 CMD */
    _cmd_st cmd_queue;
    u8 has_cmd_queue_b;
} _dt_st;

#endif
