#ifndef __ANO_BASE_H
#define __ANO_BASE_H

#include "main.h"
#include "ano_protocol.h"
#include "uarts.h"
#include <stdarg.h>
typedef struct stAnoBase stAnoBase;

typedef enum
{
    ano_printf_black_em = 0,
    ano_printf_red_em,
    ano_printf_green_em,
} ano_printf_color_em;

typedef struct
{
    int (*data_receive_prepare)(stAnoBase*me , uint8_t ucdata);
    int (*cmd_send)(stAnoBase*me, u8 dest_addr, u8 cid, const u8 *cmd_bytes, u8 cmd_len);
    int (*ck_back)(stAnoBase*me, u8 dest_addr, u8 id, u8 sc, u8 ac);
    int (*par_back)(stAnoBase*me, u8 dest_addr, u16 par_id, s32 par_val);
    int (*sendID_set)(stAnoBase*me,u8 ucFrame_num,u16 usFreq);
    int (*check_to_send)(stAnoBase* me,u8 ucFrame_num);
    int (*WTS_set)(stAnoBase* me,u8 ucFrame_num,uint8_t ucState);
    int (*wait_ck_clear)(stAnoBase* me);
    /* === 逐字段访问器 vtable 槽 === */
    uint8_t (*ck_id_get)(stAnoBase *me);
    uint8_t (*ck_sc_get)(stAnoBase *me);
    uint8_t (*ck_ac_get)(stAnoBase *me);
    uint8_t (*cmd_cid_get)(stAnoBase *me);
    int (*cmd_copy_bytes)(stAnoBase *me, uint8_t *buf, uint8_t max_len);
    uint16_t (*par_id_get)(stAnoBase *me);
    int32_t (*par_val_get)(stAnoBase *me);
    int (*ano_ck_back_check)(stAnoBase*me);
    int (*send_string)(stAnoBase *me, s32 lValue, char *pcstr);
    int (*ano_check_data)(stAnoBase *me,stUartBase* pstbase_uart);
    int (*ano_printf)(stAnoBase *me, ano_printf_color_em color_em, const char *fmt, va_list ap);
    int8_t (*wait_ck_get_c)(stAnoBase* me);
}stAnoOps;


struct stAnoBase
{
    stAnoOps* ops;
};

int vano_data_receive_prepare(stAnoBase *me, uint8_t ucdata);

int vano_cmd_send_v(stAnoBase *me, u8 dest_addr, u8 cid, const u8 *cmd_bytes, u8 cmd_len);

int ano_ck_back_v(stAnoBase *me, u8 dest_addr, u8 id, u8 sc, u8 ac);

int ano_par_back_v(stAnoBase *me, u8 dest_addr, u16 par_id, s32 par_val);

/* === 逐字段访问器（不经过 vtable，直接函数） === */
uint8_t ano_ck_id_get(stAnoBase *me);
uint8_t ano_ck_sc_get(stAnoBase *me);
uint8_t ano_ck_ac_get(stAnoBase *me);

uint8_t ano_cmd_cid_get(stAnoBase *me);
int ano_cmd_copy_bytes_s(stAnoBase *me, uint8_t *buf, uint8_t max_len);

uint16_t ano_par_id_get(stAnoBase *me);
int32_t ano_par_val_get(stAnoBase *me);

int vano_sendID_set(stAnoBase*me,u8 ucFrame_num,u16 usFreq);

int vano_check_to_send(stAnoBase* me,u8 ucFrame_num);

int vano_WTS_set(stAnoBase* me,u8 ucFrame_num ,uint8_t ucState);

int vano_wait_ck_clear(stAnoBase* me);

int vano_ck_back_check(stAnoBase*me);

int vano_send_string(stAnoBase *me, s32 lValue, char *pcstr);

int vano_check_data(stAnoBase *me,stUartBase* pstbase_uart);

int vano_printf(stAnoBase *me, ano_printf_color_em color_em, const char *fmt, ...);

int8_t ano_get_wait_ck_c(stAnoBase* me);

#endif
