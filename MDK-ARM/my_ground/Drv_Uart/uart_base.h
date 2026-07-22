#ifndef __UART_BASE_H
#define __UART_BASE_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

typedef struct stUartBase stUartBase;

typedef struct
{
    void (* transmit)(stUartBase *me,uint8_t *pucData,uint16_t usLen);
    //, uint8_t* pdata,uint16_t size
    void (* receive)(stUartBase *me);
    void (* fifoinit)(stUartBase *me);
    void (* fifoadd)(stUartBase *me ,uint16_t usLen);
    UART_HandleTypeDef* (*uart_handle_get)(stUartBase* me);
    uint8_t (* read_ucbyte)(stUartBase* stBase,uint8_t* pucdata);
    void (* transmit_debug)(stUartBase *me,uint8_t *pucData,uint16_t usLen);
    void (*uart_Ring_tx_complete_isr)(stUartBase* stBase);

}stUartOps;

struct stUartBase
{
  const stUartOps* ops;
};
void uart_transmit(stUartBase* me,uint8_t *pucData,uint16_t usLen);
 
void uart_receive_enable(stUartBase* me);

void uart_fifo_init(stUartBase* me);

void uart_fifo_add(stUartBase *me, uint16_t usLen);

UART_HandleTypeDef* uart_handle_get(stUartBase* me);

uint8_t uc_uart_read_ucbyte(stUartBase *me, uint8_t *pucdata);

void uart_transmit_debug(stUartBase *me,uint8_t *pucData,uint16_t usLen);

void uart_Ring_tx_complete_isr(stUartBase* me);

#endif
