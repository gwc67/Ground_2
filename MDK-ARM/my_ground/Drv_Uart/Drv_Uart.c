/**
 * Drv_Uart.c — 地面站 HAL UART 回调（STM32F407 适配）
 *
 * 本文件是"HAL 库回调 → 自研 vtable"的桥梁。新手重点理解这里：
 *   单片机的串口数据是硬件随时来的，HAL 库规定了一些"回调函数"，
 *   在特定事件发生时由 HAL 自动调用。我们要做的就是把这些回调
 *   转发到自己的 vtable，再分发到对应串口的实现。
 *
 * 处理三个串口：
 *   USART1 (pstbase_ground_uart) — 连接飞控，ANO 协议，DMA TX + DMA RX
 *   USART2 (pstbase_screen_uart) — 串口屏幕，纯字节收发，DMA TX + DMA RX
 *   USART3 (pstbase_usart3_uart) — ANO 协议，IT TX + IT RX
 */
#include "Drv_Uart.h"
#include "cmsis_os.h"
#include "usart.h"
#include "uart_true.h"
#include "uarts.h"
#include "ano.h"
#include "ano_device_ground.h"
#include "ano_device_usart3.h"
#include "touch_uart/touch_uart.h"
#include "Mission/mission_planner.h"
#include <stdio.h>

/* ============== 错误恢复回调 ==============
 * 串口遇到噪声/过载/校验错误时 HAL 调本函数。错误会让接收卡死，
 * 必须清标志 + 中止当前接收 + 重新使能，否则这个串口就"哑"了。
 * 每个 UART 各一段，逻辑相同：清 ORE/NE/PE/FE 标志 → AbortReceive → 重新 receive。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_handle_get(pstbase_ground_uart))
    {
        /* F4 清除错误标志：读 SR 后读 DR（HAL 宏内部处理） */
        __HAL_UART_CLEAR_OREFLAG(uart_handle_get(pstbase_ground_uart));
        __HAL_UART_CLEAR_NEFLAG(uart_handle_get(pstbase_ground_uart));
        __HAL_UART_CLEAR_PEFLAG(uart_handle_get(pstbase_ground_uart));
        __HAL_UART_CLEAR_FEFLAG(uart_handle_get(pstbase_ground_uart));
        HAL_UART_AbortReceive_IT(uart_handle_get(pstbase_ground_uart));
        uart_receive_enable(pstbase_ground_uart);
    }
    if (huart == uart_handle_get(pstbase_screen_uart))
    {
        __HAL_UART_CLEAR_OREFLAG(uart_handle_get(pstbase_screen_uart));
        __HAL_UART_CLEAR_NEFLAG(uart_handle_get(pstbase_screen_uart));
        __HAL_UART_CLEAR_PEFLAG(uart_handle_get(pstbase_screen_uart));
        __HAL_UART_CLEAR_FEFLAG(uart_handle_get(pstbase_screen_uart));
        HAL_UART_AbortReceive_IT(uart_handle_get(pstbase_screen_uart));
        uart_receive_enable(pstbase_screen_uart);
    }
    if (huart == uart_handle_get(pstbase_usart3_uart))
    {
        __HAL_UART_CLEAR_OREFLAG(uart_handle_get(pstbase_usart3_uart));
        __HAL_UART_CLEAR_NEFLAG(uart_handle_get(pstbase_usart3_uart));
        __HAL_UART_CLEAR_PEFLAG(uart_handle_get(pstbase_usart3_uart));
        __HAL_UART_CLEAR_FEFLAG(uart_handle_get(pstbase_usart3_uart));
        HAL_UART_AbortReceive_IT(uart_handle_get(pstbase_usart3_uart));
        uart_receive_enable(pstbase_usart3_uart);
    }
}

/* ============== 接收事件回调 ==============
 * ★ 收数据的入口。DMA/IT 收到一批字节并遇到"空闲"时，HAL 调本函数，
 * Size = 这次收到的字节数。这里调 uart_fifo_add 把这批字节转存进 RingBuffer。
 * 注意 USART3 是 IT 模式，收完一批后必须重新 receive 才能收下一批
 * (DMA 循环模式则会自动续收，不用 re-enable)。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == uart_handle_get(pstbase_ground_uart))
    {
        uart_fifo_add(pstbase_ground_uart, Size);
        /* DMA 循环模式自动续收，无需 re-enable */
    }
    if (huart == uart_handle_get(pstbase_screen_uart))
    {
        uart_fifo_add(pstbase_screen_uart, Size);
        uart_receive_enable(pstbase_screen_uart);
    }
    if (huart == uart_handle_get(pstbase_usart3_uart))
    {
        uart_fifo_add(pstbase_usart3_uart, Size);
        /* IT 模式需要重新使能接收 */
        uart_receive_enable(pstbase_usart3_uart);
    }
}

/* ============== 接收完成回调（预留） ============== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

/* ============== 发送完成回调 ==============
 * ★ 非阻塞发送的关键：一帧硬件发完，HAL 调本函数。这里经 vtable 转发到
 * uart_Ring_tx_complete_isr_DMA/IT，让它出队下一帧续发。没有这个回调，
 * 队列里排队的帧就永远发不出去(只会发第一帧)。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_handle_get(pstbase_ground_uart))
        uart_Ring_tx_complete_isr(pstbase_ground_uart);
    if (huart == uart_handle_get(pstbase_screen_uart))
        uart_Ring_tx_complete_isr(pstbase_screen_uart);
    if (huart == uart_handle_get(pstbase_usart3_uart))
        uart_Ring_tx_complete_isr(pstbase_usart3_uart);
}

/* ============== 数据轮询 ==============
 * ★ 任务循环(Ano_Scheduler.c)周期调本函数消费数据。
 * 这里把"从 UART 读字节"和"协议解析"串起来：
 *   vano_check_data(ano设备, uart句柄)
 *     → ANO 层 while(uc_uart_read_ucbyte(...)) 从 RingBuffer 读字节
 *     → 喂给 ANO 7级状态机，收满一帧就处理。
 * screen 串口走另一套(touch_uart)，因为它不是 ANO 协议而是 ASCII 命令。
 * PYTHON_DEBUG 宏在编译期切换屏幕协议实现。 */
void DrvUartDataCheck(void)
{
    /* USART1 — DMA 接收，ANO 协议 */
    vano_check_data(pstAnobase_Ground, pstbase_ground_uart);
    /* USART3 — IT 接收，ANO 协议 */
    vano_check_data(pstAnobase_Usart3, pstbase_usart3_uart);
    /* USART2 — 屏幕协议（编译期切换） */
#if PYTHON_DEBUG
    screen_uart_check_python();
#else
    screen_uart_check_touch();    /* 被动：收屏幕命令 */
    // screen_uart_check_phase();    /* 主动：轮询 FC 相位 → 推 UI 更新 */
#endif
}
