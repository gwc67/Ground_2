/**
 * uart_log.c — 平台日志层实现（Linux 风格）
 *
 * 后端走 uart_transmit（原始字节流），不封装任何协议帧。
 * 通过 uart_printf_v 统一入口，宏封装自动拼接级别前缀 [DBG]/[INF]/[WRN]/[ERR]。
 *
 * 替换后端：只需修改本文件的 uart_printf_v 实现，调用方无需改动。
 */
#include "uart_log.h"
#include "uart_base.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define UART_LOG_BUF_SIZE  128

int uart_printf_v(stUartBase *uart, int level, const char *fmt, ...)
{
    if (uart == NULL || fmt == NULL) return -1;

    /* 级别合法性（保留扩展，当前未使用 level 做过滤） */
    (void)level;

    char buf[UART_LOG_BUF_SIZE];

    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len < 0) return -1;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

    /* uart_transmit 通过 TX 队列非阻塞发送，内部会 memcpy 到队列项 */
    uart_transmit(uart, (uint8_t *)buf, (uint16_t)len);
    return len;
}
