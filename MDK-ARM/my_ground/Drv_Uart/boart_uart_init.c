#include "main.h"
#include "usart.h"
#include "uart_true.h"
#include "ring_buffer.h"
#include "driver_registry.h"
#include "uarts.h"

/* ============== Buffer 配置 ============== */

/* USART1 — FC 通信（ANO 协议），DMA TX + DMA RX 循环 */
#define GROUND_RXBufferSize 15
#define GROUND_RXFIFOBufferSize (GROUND_RXBufferSize * 40)

/* USART2 — 串口屏幕，DMA TX + DMA RX 循环 */
#define SCREEN_RXBufferSize 64
#define SCREEN_RXFIFOBufferSize (SCREEN_RXBufferSize * 40)

/* USART3 — IT TX + IT RX，ANO 协议 */
#define USART3_RXBufferSize 15
#define USART3_RXFIFOBufferSize (USART3_RXBufferSize * 60)

/* F407 DMA 可直接访问普通 SRAM，不需要 .dma_buf section */
static uint8_t Ground_RxBuffer[GROUND_RXBufferSize];
static uint8_t Ground_TxBuffer[256];
static uint8_t Ground_RxFIFOBuffer[GROUND_RXFIFOBufferSize];

static uint8_t Screen_RxBuffer[SCREEN_RXBufferSize];
static uint8_t Screen_TxBuffer[256];
static uint8_t Screen_RxFIFOBuffer[SCREEN_RXFIFOBufferSize];

static uint8_t USART3_RxBuffer[USART3_RXBufferSize];
static uint8_t USART3_TxBuffer[64];
static uint8_t USART3_RxFIFOBuffer[USART3_RXFIFOBufferSize];

/* ============== 全局指针 ============== */

/* USART1 — 连接飞控 USART6，ANO 协议通道 */
stUartBase* pstbase_ground_uart;

/* USART2 — 串口屏幕，纯 UART vtable，无 ANO 协议 */
stUartBase* pstbase_screen_uart;

/* USART3 — IT TX + IT RX，ANO 协议通道 */
stUartBase* pstbase_usart3_uart;

/* ============== 实例 ============== */

static stUartRing ground_uart;
static stUartRing screen_uart;
static stUartRing usart3_uart;

/* boart_uart_init.c — Board 层，全工程唯一"实例化 UART"的地方。
 *
 * 这里干三件事(每个串口一遍)：
 *   1) 分配收发 buffer 数组(static，全局静态存储)
 *   2) 填 stUartRingConfig 调 uartRing_init —— 绑 HAL 句柄、buffer、
 *      选 DMA/IT(ucRxIsIT/ucTxIsDMA)，init 内部据此挂对应 ops 表
 *   3) 暴露基类指针 pstbase_xxx_uart 给全工程用(外部只认基类指针)
 *
 * ★ F407 说明：F407 的 DMA 能直接访问普通 SRAM，所以 TX/RX buffer
 *   就是普通 static 数组，不需要放专门的 .dma_buf 段，也不用手动
 *   清 D-Cache。(部分 CLAUDE.md 里写的 .dma_buf/cache-safe 是早期
 *   H7 芯片的描述，对 F4 不适用，以这里代码为准。)
 *
 * 本函数由 DRIVER_INIT_1 注册，排在 ano_board_init(b2) 之前执行 ——
 * 因为 ANO 层发数据要用到已建好的 UART 实例。 */
void uart_board_init(void)
{
    /* USART1 — FC 通信：DMA TX + DMA RX 循环 */
    uartRing_init(&ground_uart, &(stUartRingConfig){
        .pUartHandle      = &huart1,
        .pucRxbuffer      = Ground_RxBuffer,
        .usRxbuffer_size  = GROUND_RXBufferSize,
        .pucRingBuffer    = Ground_RxFIFOBuffer,
        .ulRingBufferLen  = GROUND_RXFIFOBufferSize,
        .pTxDmabuffer     = Ground_TxBuffer,
        .ucRxIsIT         = 0,    /* DMA 循环接收 */
        .ucTxIsDMA        = 1,    /* DMA 发送（非阻塞队列） */
    });

    /* USART2 — 串口屏幕：DMA TX + DMA RX 循环 */
    uartRing_init(&screen_uart, &(stUartRingConfig){
        .pUartHandle      = &huart2,
        .pucRxbuffer      = Screen_RxBuffer,
        .usRxbuffer_size  = SCREEN_RXBufferSize,
        .pucRingBuffer    = Screen_RxFIFOBuffer,
        .ulRingBufferLen  = SCREEN_RXFIFOBufferSize,
        .pTxDmabuffer     = Screen_TxBuffer,
        .ucRxIsIT         = 0,    /* DMA 循环接收 */
        .ucTxIsDMA        = 1,    /* DMA 发送（非阻塞队列） */
        .ucRxIsIdle       = 1,
    });

    /* USART3 — 与飞控通信：IT TX + IT RX，ANO 协议 */
    uartRing_init(&usart3_uart, &(stUartRingConfig){
        .pUartHandle      = &huart3,
        .pucRxbuffer      = USART3_RxBuffer,
        .usRxbuffer_size  = USART3_RXBufferSize,
        .pucRingBuffer    = USART3_RxFIFOBuffer,
        .ulRingBufferLen  = USART3_RXFIFOBufferSize,
        .pTxDmabuffer     = USART3_TxBuffer,
        .ucRxIsIT         = 1,    /* IT 接收（逐字节中断） */
        .ucTxIsDMA        = 0,    /* IT 发送（阻塞队列） */
    });

    /* 暴露基类指针 */
    pstbase_ground_uart = &ground_uart.stBase;
    pstbase_screen_uart = &screen_uart.stBase;
    pstbase_usart3_uart = &usart3_uart.stBase;
    uart_receive_enable(pstbase_ground_uart);
    uart_receive_enable(pstbase_screen_uart);
    uart_receive_enable(pstbase_usart3_uart);
}

/* uart_board_init 先于 ano_board_init 执行（b1 < b2） */
DRIVER_INIT_1(uart_board_init);
