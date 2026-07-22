/* uart_base.c — UART vtable 的"薄分发层"。
 * 每个函数就两行：assert 检查 ops 槽非空 → 转发到具体实现。
 * 外部统一调 uart_transmit/uart_fifo_add/... 这些函数名，
 * 不用关心底层是 DMA 还是 IT —— 实现差异由 ops 表决定(多态)。
 * 新手不用细读，知道"这里只是中转"即可，真正逻辑在 uart_true.c。 */
#include "uart_base.h"

// 这些额外参数的确定是由具体上层用到了再说
void uart_transmit(stUartBase* me,uint8_t *pucData,uint16_t usLen)
{
    assert_param(me->ops->transmit);
    me->ops->transmit(me,pucData,usLen);
}

void uart_receive_enable(stUartBase* me)
{
    assert_param(me->ops->receive);
    me->ops->receive(me);
}

void uart_fifo_init(stUartBase* me)  
{
    // assert_param(me->ops->fifoinit);
    if (me->ops->fifoinit)
    {
        me->ops->fifoinit(me);
    }
    
}
void uart_fifo_add(stUartBase* me,uint16_t usLen)
{
    assert_param(me->ops->fifoadd);
    me->ops->fifoadd(me,usLen);
}
UART_HandleTypeDef* uart_handle_get(stUartBase* me)
{
    assert_param(me->ops->uart_handle_get);
    return me->ops->uart_handle_get(me);
}

uint8_t uc_uart_read_ucbyte(stUartBase* me,uint8_t* pucdata)
{
    assert_param(me->ops->read_ucbyte);
    return me->ops->read_ucbyte(me, pucdata);
}
void uart_transmit_debug(stUartBase *me,uint8_t *pucData,uint16_t usLen)
{
    assert_param(me->ops->transmit_debug);
    me->ops->transmit_debug(me,pucData,usLen);
}
void uart_Ring_tx_complete_isr(stUartBase* me)
{
    assert_param(me->ops->uart_Ring_tx_complete_isr);
    me->ops->uart_Ring_tx_complete_isr(me);
}
