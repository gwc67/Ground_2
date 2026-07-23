#ifndef __TOUCH_UART_H
#define __TOUCH_UART_H

#include "mission_planner.h"

#if PYTHON_DEBUG

/**
 * @brief 解析 Python 上位机命令（USART2）
 *
 * 命令：MAP: / REQUEST_PATH
 * 调用：DrvUartDataCheck()
 */
void screen_uart_check_python(void);

#else

/* ── 屏幕 UI 显示模式 ── */
typedef enum {
    UI_MODE_PREVIEW,    /* Patrol + Return 同时显示 */
    UI_MODE_PATROL,     /* 仅 Patrol */
    UI_MODE_RETURN,     /* 仅 Return */
} ui_mode_t;
/**
 * @brief 解析真实串口屏命令（USART2）
 *
 * 命令：zone: / request_route
 * 调用：DrvUartDataCheck()
 */
void screen_uart_check_touch(void);

bool screen_begin_fly_b(void);

void screen_set_ui_mode(ui_mode_t mode);

void screen_send_delivery(void);

bool request_route_b(void);

#endif

#endif /* __TOUCH_UART_H */
