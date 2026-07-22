#ifndef __UARTS_H
#define __UARTS_H

#include "uart_base.h"

/* USART1 — ANO 协议通道，连接飞控 USART6 */
extern stUartBase *pstbase_ground_uart;

/* USART2 — 串口屏幕，纯 UART vtable，无 ANO 协议 */
extern stUartBase *pstbase_screen_uart;

/* USART3 — ANO 协议通道，IT TX + IT RX */
extern stUartBase *pstbase_usart3_uart;

void uart_board_init(void);

#endif
