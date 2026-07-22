#include "ano_base.h"
#include <stdarg.h>

int vano_data_receive_prepare(stAnoBase *me, uint8_t ucdata)
{
    assert_param(me->ops->data_receive_prepare);
    return me->ops->data_receive_prepare(me, ucdata);
}

int vano_cmd_send_v(stAnoBase *me, u8 dest_addr, u8 cid, const u8 *cmd_bytes, u8 cmd_len)
{
    assert_param(me->ops->cmd_send);
    return me->ops->cmd_send(me, dest_addr, cid, cmd_bytes, cmd_len);
}

int ano_ck_back_v(stAnoBase *me, u8 dest_addr, u8 id, u8 sc, u8 ac)
{
    assert_param(me->ops->ck_back);
    return me->ops->ck_back(me, dest_addr, id, sc, ac);
}

int ano_par_back_v(stAnoBase *me, u8 dest_addr, u16 par_id, s32 par_val)
{
    assert_param(me->ops->par_back);
    return me->ops->par_back(me, dest_addr, par_id, par_val);
}

int vano_sendID_set(stAnoBase*me,u8 ucFrame_num,u16 usFreq)
{
    assert_param(me->ops->sendID_set);
    return me->ops->sendID_set(me,ucFrame_num,usFreq);
}

int vano_check_to_send(stAnoBase* me,u8 ucFrame_num)
{
    assert_param(me->ops->check_to_send);
    return me->ops->check_to_send(me,ucFrame_num);
}

int vano_WTS_set(stAnoBase* me,u8 ucFrame_num ,uint8_t ucState)
{
    assert_param(me->ops->WTS_set);
    return me->ops->WTS_set(me,ucFrame_num,ucState);
}

int vano_wait_ck_clear(stAnoBase* me)
{
    assert_param(me->ops->wait_ck_clear);
    return me->ops->wait_ck_clear(me);
}

/* === 逐字段访问器 dispatcher === */

uint8_t ano_ck_id_get(stAnoBase *me)
{
    assert_param(me->ops->ck_id_get);
    return me->ops->ck_id_get(me);
}

uint8_t ano_ck_sc_get(stAnoBase *me)
{
    assert_param(me->ops->ck_sc_get);
    return me->ops->ck_sc_get(me);
}

uint8_t ano_ck_ac_get(stAnoBase *me)
{
    assert_param(me->ops->ck_ac_get);
    return me->ops->ck_ac_get(me);
}

uint8_t ano_cmd_cid_get(stAnoBase *me)
{
    assert_param(me->ops->cmd_cid_get);
    return me->ops->cmd_cid_get(me);
}

int ano_cmd_copy_bytes_s(stAnoBase *me, uint8_t *buf, uint8_t max_len)
{
    assert_param(me->ops->cmd_copy_bytes);
    return me->ops->cmd_copy_bytes(me, buf, max_len);
}

uint16_t ano_par_id_get(stAnoBase *me)
{
    assert_param(me->ops->par_id_get);
    return me->ops->par_id_get(me);
}

int32_t ano_par_val_get(stAnoBase *me)
{
    assert_param(me->ops->par_val_get);
    return me->ops->par_val_get(me);
}

int vano_ck_back_check(stAnoBase*me)
{
    assert_param(me->ops->ano_ck_back_check);
    return me->ops->ano_ck_back_check(me);
}

int vano_send_string(stAnoBase *me, s32 lValue, char *pcstr)
{
    assert_param(me->ops->send_string);
    return me->ops->send_string(me,lValue,pcstr);
}

int vano_check_data(stAnoBase *me,stUartBase* pstbase_uart)
{
    assert_param(me->ops->ano_check_data);
    return me->ops->ano_check_data(me,pstbase_uart);
}

int vano_printf(stAnoBase *me, ano_printf_color_em color_em, const char *fmt, ...)
{
    assert_param(me->ops->ano_printf);

    va_list args;
    va_start(args, fmt);
    int ret = me->ops->ano_printf(me, color_em, fmt, args);
    va_end(args);
    return ret;
}

int8_t ano_get_wait_ck_c(stAnoBase* me)
{
    if (me->ops->wait_ck_get_c)
    {
        return (int8_t)me->ops->wait_ck_get_c(me);
    }
    return -1;
}

