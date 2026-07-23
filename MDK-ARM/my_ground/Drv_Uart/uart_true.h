#ifndef __UART_TRUE_H
#define __UART_TRUE_H

#include "main.h"
#include "usart.h"
#include "uart_base.h"
#include "ring_buffer.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"

typedef struct
{
    uint8_t data[64];
    uint16_t len;
} tx_queue_item_t;

typedef struct
{
    stUartBase stBase;
    UART_HandleTypeDef* pUartHandle;
    uint8_t* pucRxbuffer;
    uint16_t usRxbuffer_size;

    stRingBufTdf stdRingbuffer;          // 值类型，嵌入结构体（非指针）

    QueueHandle_t xTxQuene_t;
    volatile uint8_t TxInProgress_vuc;
    uint8_t* TxDmabuffer_puc;
    volatile uint16_t usLastRxPos;
} stUartRing;
;
typedef struct {
    UART_HandleTypeDef*   pUartHandle;
    uint8_t*              pucRxbuffer;
    uint16_t              usRxbuffer_size;
    uint8_t*              pucRingBuffer;
    uint32_t              ulRingBufferLen;
    uint8_t*              pTxDmabuffer;      // 持久 TX buffer（DMA 需 .dma_buf 段，IT 用普通 RAM）
    uint8_t               ucRxIsIT;          // 1 = IT 接收, 0 = DMA 循环接收
    uint8_t               ucTxIsDMA;         // 1 = DMA 发送, 0 = IT 发送（队列非阻塞）
    uint8_t               ucRxIsIdle;
} stUartRingConfig;

void uartRing_init(stUartRing *me, const stUartRingConfig *cfg);
void uart_init_it(stUartRing *me, const stUartRingConfig *cfg);

#endif
