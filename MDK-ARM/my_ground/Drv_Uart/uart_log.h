/**
 * uart_log.h — 平台日志层（Linux 风格）
 *
 * 参照 Linux printk / dev_xxx 模式，提供分级日志接口。
 * 后端默认走 uart_transmit（原始数据流，无协议帧封装），
 * 将来可替换为 RTT / semihosting / 文件系统而调用方无需改动。
 *
 * 使用示例：
 *   uart_dbg(pstbase_ground_uart, "radar x=%d y=%d\r\n", x, y);
 *   uart_info(pstbase_screen_uart, "battery: %u.%02uV\r\n", v/100, v%100);
 *   uart_warn(pstbase_usart3_uart, "queue full\r\n");
 *   uart_err(pstbase_ground_uart, "crc failed, cmd=0x%02x\r\n", cmd);
 *
 * 注意：
 *   - 调用方需自行添加 \r\n（接口不自动追加）
 *   - 单条消息最大 127 字节（含 '\0'），超长被截断
 *   - TX 队列满（8 帧）时丢弃，不阻塞调用方
 */
#ifndef __UART_LOG_H
#define __UART_LOG_H

#include "uart_base.h"

/* ============== 日志级别 ============== */

#define UART_LOG_DEBUG  0
#define UART_LOG_INFO   1
#define UART_LOG_WARN   2
#define UART_LOG_ERR    3

/* ============== 核心接口 ============== */

/**
 * uart_printf_v — 格式化并发送分级日志（原始字节流，无协议封装）
 *
 * @param uart   UART 句柄（stUartBase*）
 * @param level  日志级别（UART_LOG_xxx）
 * @param fmt    格式化字符串（printf 风格）
 * @return       实际发送字节数，失败返回负值
 */
int uart_printf_v(stUartBase *uart, int level, const char *fmt, ...);

/* ============== 宏封装（自动拼接级别前缀） ============== */

#define uart_dbg(uart, fmt, ...) \
    uart_printf_v((uart), UART_LOG_DEBUG, "[DBG] " fmt, ##__VA_ARGS__)

#define uart_info(uart, fmt, ...) \
    uart_printf_v((uart), UART_LOG_INFO,  "[INF] " fmt, ##__VA_ARGS__)

#define uart_warn(uart, fmt, ...) \
    uart_printf_v((uart), UART_LOG_WARN,  "[WRN] " fmt, ##__VA_ARGS__)

#define uart_err(uart, fmt, ...) \
    uart_printf_v((uart), UART_LOG_ERR,   "[ERR] " fmt, ##__VA_ARGS__)

#endif /* __UART_LOG_H */
