#include "uart_true.h"
#include "uarts.h"
#include "cmsis_os2.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "semphr.h"
#include "usart.h"
#include "string.h"
/* container_of：从基类指针 stUartBase* 还原出子类指针 stUartRing*。
 * 前提：stUartRing 第一个成员必须是 stBase（见 uart_true.h:20）。
 * vtable 函数收到的都是基类指针，要访问子类私有字段(如 pUartHandle、
 * xTxQuene_t)就得靠这个宏转回去。理解成 C++ 的 this 还原。 */
#define container_of(ptr, type, member) ((type*)((char*)(ptr) - offsetof(type, member)))
/* TX 队列容量：可缓存 32 帧。满了再发会丢帧（设计取舍，协议层不关心）。 */
#define TX_QUENE_LEN 8
/* 注意：xQueueCreate 实际用的是 32（见 uart_Ring_tx_initQuene），这里这个
 * 宏名保留但未直接用于创建，仅作配置参考。真正的容量在 initQuene 里硬编码 32。 */
//真正的工业项目这些变量都应该会在platform层经过重新定义，保持原来的变量类型不变

/* ============================================================================
 * 本文件 = UART vtable 的"子类实现"。
 *
 * 每个 stUartRing 实例代表一个串口，挂一张 ops 表决定它用 DMA 还是 IT。
 * 两套核心机制（新手重点理解）：
 *
 * 1) 接收：DMA/IT 把字节搬进 pucRxbuffer，HAL 回调里调 uart_ring_add()
 *    把字节转存进 RingBuffer(解耦"收"和"处理"的速度差)。
 *    上层通过 uart_ring_read()(=ops.read_ucbyte) 逐个读出。
 *
 * 2) 发送(非阻塞 TX 队列)：
 *    任务级 uart_Ring_tx_DMA_Queue：帧拷进队列 → 若空闲就启动 DMA → 返回不等
 *    ISR 级 uart_Ring_tx_complete_isr_DMA：DMA 发完中断里出队下一帧续发
 *    两级共用 xTxQuene_t 和 TxInProgress_vuc(volatile，跨任务/ISR 共享)
 *
 * 见底部 4 张 ops 表：dma_ops / it_ops / dma_it_ops / dma_rx_only_ops
 * 分别对应"DMA收发 / IT收发 / DMA发+IT收 / IT发+DMA收"四种组合。
 * ========================================================================== */

/// @brief IT 非阻塞发送（队列 + 门卫，与 DMA_Queue 同模式）
/// 与 uart_Ring_tx_DMA_Queue 结构完全一样，只是底层用 HAL_UART_Transmit_IT。
/// "非阻塞"含义：帧入队后立即返回，不等硬件发完；发完中断里再出队续发。
static void uart_Ring_tx_IT(stUartBase *stBase,uint8_t *pucData,uint16_t usLen)
{
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    /* 帧 > 64 或空 → 直接拒收。64 是 tx_queue_item_t.data 的硬上限。 */
    if (usLen > 64 || usLen == 0) return;

    /* 拷贝到栈上临时变量，再入队。入队时 FreeRTOS 会再拷贝一份到队列内部。
     * 0 超时：队列满就立即返回失败(丢这帧)，不阻塞调用方。 */
    tx_queue_item_t item;
    item.len = usLen;
    memcpy(item.data, pucData, usLen);

    if (xQueueSend(me->xTxQuene_t, &item, 0) != pdPASS) return;

    /* 门卫 TxInProgress_vuc：0=硬件空闲，1=正在发。
     * 如果当前没在发，就从队列取出刚入的帧启动发送。
     * 如果正在发，啥也不做——发完中断会自动出队续发。 */
    if (!me->TxInProgress_vuc)
    {
        tx_queue_item_t out;
        if (xQueueReceive(me->xTxQuene_t, &out, 0) == pdTRUE)
        {
            /* 拷到持久 TX buffer(TxDmabuffer_puc)，硬件发的是这块内存。
             * 注意：不能直接发 out.data(栈变量)，因为函数返回后栈就没了。 */
            memcpy(me->TxDmabuffer_puc, out.data, out.len);
            me->TxInProgress_vuc = 1;
            HAL_UART_Transmit_IT(me->pUartHandle, me->TxDmabuffer_puc, out.len);
        }
    }
}

static void uart_Ring_rx_IT(stUartBase *stBase)
{   
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    HAL_UARTEx_ReceiveToIdle_IT(me->pUartHandle,me->pucRxbuffer,me->usRxbuffer_size);
}
// IT 发送完成 ISR：DMA/IT 发完一帧，硬件调 HAL_UART_TxCpltCallback → 本函数。
// 职责：出队下一帧续发；队列空了就标记空闲，让下次任务级发送能启动。
// 注意：在 ISR 上下文，必须用 FromISR 版本的 FreeRTOS API，不能阻塞。
static void uart_Ring_tx_complete_isr_IT(stUartBase *stBase)
{
    stUartRing *me = container_of(stBase, stUartRing, stBase);
    BaseType_t xWoken = pdFALSE;
    tx_queue_item_t item;
    if (xQueueReceiveFromISR(me->xTxQuene_t, &item, &xWoken) == pdTRUE)
    {
        /* 队列还有帧 → 拷到 TX buffer 续发 */
        memcpy(me->TxDmabuffer_puc, item.data, item.len);
        HAL_UART_Transmit_IT(me->pUartHandle, me->TxDmabuffer_puc, item.len);
    }
    else
    {
        /* 队列空 → 标记空闲。下次任务级 transmit 进来时会重新启动发送。 */
        me->TxInProgress_vuc = 0;
    }
    /* 如果这次出队唤醒了某个高优先级任务，退出 ISR 时强制切换 */
    if (xWoken) portYIELD_FROM_ISR(xWoken);
}

static void uart_Ring_tx_initQuene(stUartRing *me)
{
    me->xTxQuene_t = xQueueCreate(40,sizeof(tx_queue_item_t));
    me->TxInProgress_vuc = 0;
}


/* DMA 发送完成 ISR —— 结构和 IT 版完全一样，只是底层换 DMA。
 * 被调用链：硬件发完 → HAL_UART_TxCpltCallback(Drv_Uart.c) →
 *           uart_Ring_tx_complete_isr(uart_base.c) → 本函数。
 * 出队续发，队列空则置 TxInProgress=0。 */
static void uart_Ring_tx_complete_isr_DMA(stUartBase* stBase)
{
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    BaseType_t xWoken = pdFALSE;
    tx_queue_item_t item;
    if (xQueueReceiveFromISR(me->xTxQuene_t, &item, &xWoken) == pdTRUE)
    {
          // 队列中还有帧，启动下一帧
          memcpy(me->TxDmabuffer_puc, item.data, item.len);
          HAL_UART_Transmit_DMA(me->pUartHandle, me->TxDmabuffer_puc, item.len);
    }
    else
    {
          // 队列空了，DMA 空闲
          me->TxInProgress_vuc = 0;
    }
}

/* DMA 非阻塞发送 —— 本工程主用模式(USART1/USART2)。
 * 和 uart_Ring_tx_IT 逐行对应，仅 HAL_UART_Transmit_DMA 不同。
 * 三步：①拷帧入队(满则丢) ②若空闲则出队拷到TX buffer启动DMA ③立即返回
 * 硬件发完由 uart_Ring_tx_complete_isr_DMA 出队续发。 */
static void uart_Ring_tx_DMA_Queue(stUartBase *stBase,uint8_t *pucData,uint16_t usLen)
{
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    if (usLen > 64 || usLen == 0)
    {
        return;
    }
    tx_queue_item_t item;
    item.len = usLen;
    memcpy(item.data,pucData,usLen);

    //入队
    if (xQueueSend(me->xTxQuene_t,&item,0) != pdPASS) return ;
    
    //
    if (!me->TxInProgress_vuc)
    {
        tx_queue_item_t out;
        if (xQueueReceive(me->xTxQuene_t,&out,0) == pdTRUE)
        {
            memcpy(me->TxDmabuffer_puc,out.data,out.len);
            me->TxInProgress_vuc = 1;
            HAL_UART_Transmit_DMA(me->pUartHandle,me->TxDmabuffer_puc,out.len);
        }
    }
}

/* 启用 DMA 循环接收。
 * HAL_UARTEx_ReceiveToIdle_DMA：DMA 把字节搬进 pucRxbuffer，收到"空闲"
 * (线路静默一段时间)时触发 HAL_UARTEx_RxEventCallback 回调并告知 Size。
 * __HAL_DMA_DISABLE_IT(...DMA_IT_HT)：关闭"半满中断"——我们只要"空闲"
 * 通知，不要"收到一半"通知，减少中断次数。
 * 循环模式：DMA 搬满缓冲区会自动从头继续搬，不用每次手动 re-enable。 */
static void uart_Ring_rx_DMA(stUartBase *stBase)
{   
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    HAL_UARTEx_ReceiveToIdle_DMA(me->pUartHandle,me->pucRxbuffer,me->usRxbuffer_size);
    __HAL_DMA_DISABLE_IT(me->pUartHandle->hdmarx,DMA_IT_HT); // 关闭接受过半中断
}

/* 接收事件回调里调用(Drv_Uart.c 的 HAL_UARTEx_RxEventCallback)。
 * 把 DMA 刚搬进 pucRxbuffer 的 usLen 个字节逐个写进 RingBuffer。
 * 为什么不直接给上层用 pucRxbuffer？因为 DMA 还在循环往里写新数据，
 * 上层处理慢的话会被覆盖。RingBuffer 起到"缓冲+解耦速度"的作用。
 * 写法是逐字节 ucRingBufWrite，简单但够用(载荷一般不大)。 */
static void uart_ring_add(stUartBase* stBase,uint16_t usLen)
{
    stUartRing *me = container_of(stBase, stUartRing, stBase);
    for (uint16_t i = 0; i < usLen; i++)
    {
        ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
    }
}
/* 从 RingBuffer 读一个字节。返回 0=成功读到，非0=空。
 * 这是"两层 vtable 咬合点 1"的 UART 侧实现：ANO 层的 Data_Check_Ano_s
 * 通过 uc_uart_read_ucbyte→本函数 逐字节要数据喂状态机。 */
static uint8_t uart_ring_read(stUartBase* stBase,uint8_t* pucdata)
{
    stUartRing *me = container_of(stBase, stUartRing, stBase);
    return ucRingBufRead(&me->stdRingbuffer,pucdata);
}

/// @brief 返回uart_handle 句柄
/// @param stBase
/// @return
static UART_HandleTypeDef* uart_handle_get(stUartBase* stBase)
{
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    return me->pUartHandle;
}

static void uart_Ring_tx_debugger(stUartBase* stBase ,uint8_t *pucData,uint16_t usLen)
{
    stUartRing* me = container_of(stBase,stUartRing,stBase);
    HAL_UART_Transmit(me->pUartHandle,pucData,usLen,HAL_MAX_DELAY);
}
// static void uart_ring_add_dma_idle(stUartBase* stBase, uint16_t usLen)
// {
//     stUartRing *me = container_of(stBase, stUartRing, stBase);
//     uint16_t usLastPos = me->usLastRxPos;

//     if (usLen >= usLastPos) {
//         /* 正常：从 usLastPos 到 usLen 是新数据 */
//         for (uint16_t i = usLastPos; i < usLen; i++) {
//             uint16_t pos = i % me->usRxbuffer_size;
//             ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[pos]);
//         }
//     } else {
//         /* DMA 回绕：先写 usLastPos 到末尾，再写 0 到 usLen */
//         for (uint16_t i = usLastPos; i < me->usRxbuffer_size; i++) {
//             ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
//         }
//         for (uint16_t i = 0; i < usLen; i++) {
//             ucRingBufWrite(&me->stdRingbuffer, me->pucRxbuffer[i]);
//         }
//     }

//     me->usLastRxPos = usLen;
// }
/* 4 张 ops 表 —— 不同串口挂不同表，决定收发用 DMA 还是 IT。
 * 选哪张表在 uartRing_init 里根据 cfg->ucRxIsIT / ucTxIsDMA 决定。 */

/* IT 收 + IT 发（队列非阻塞）。USART3 用这套。 */
const static stUartOps usart_ring_it_ops = {
    .transmit = uart_Ring_tx_IT,
    .receive = uart_Ring_rx_IT,
    .fifoadd = uart_ring_add,
    .uart_handle_get = uart_handle_get,
    .read_ucbyte = uart_ring_read,
    .transmit_debug = uart_Ring_tx_debugger,
    .uart_Ring_tx_complete_isr = uart_Ring_tx_complete_isr_IT,
};


/* DMA 收 + DMA 发（队列非阻塞）。USART1/USART2 用这套，高速通信。 */
const static stUartOps usart_ring_dma_ops = {
    .transmit = uart_Ring_tx_DMA_Queue,
    .receive = uart_Ring_rx_DMA,
    .fifoadd = uart_ring_add,
    .uart_handle_get = uart_handle_get,
    .read_ucbyte = uart_ring_read,
    .transmit_debug = uart_Ring_tx_debugger,
    .uart_Ring_tx_complete_isr = uart_Ring_tx_complete_isr_DMA,
};

/* DMA 发 + IT 收。组合用，当前未在 board 层启用，预留。 */
const static stUartOps usart_ring_dma_it_ops = {
    .transmit = uart_Ring_tx_DMA_Queue,
    .receive = uart_Ring_rx_IT,
    .fifoadd = uart_ring_add,
    .uart_handle_get = uart_handle_get,
    .read_ucbyte = uart_ring_read,
    .transmit_debug = uart_Ring_tx_debugger,
    .uart_Ring_tx_complete_isr = uart_Ring_tx_complete_isr_DMA,
};

/* IT 发 + DMA 收。组合用，当前未在 board 层启用，预留。 */
const static stUartOps usart_ring_dma_rx_only_ops = {
    .transmit = uart_Ring_tx_IT,
    .receive = uart_Ring_rx_DMA,
    .fifoadd = uart_ring_add,
    .uart_handle_get = uart_handle_get,
    .read_ucbyte = uart_ring_read,
    .transmit_debug = uart_Ring_tx_debugger,
    .uart_Ring_tx_complete_isr = uart_Ring_tx_complete_isr_IT,
};

const static stUartOps usart_ring_tx_dma_rx_it = {
    .transmit = uart_Ring_tx_DMA_Queue,
    .receive = uart_Ring_rx_IT,
    .fifoadd = uart_ring_add,
    .uart_handle_get = uart_handle_get,
    .read_ucbyte = uart_ring_read,
    .transmit_debug = uart_Ring_tx_debugger,
    .uart_Ring_tx_complete_isr = uart_Ring_tx_complete_isr_IT,
};

/* 子类初始化 —— board 层(boart_uart_init.c)给每个串口调一次。
 * 职责：绑定 HAL 句柄、收发 buffer、RingBuffer；建 TX 队列；
 *       根据 cfg->ucRxIsIT/ucTxIsDMA 选 ops 表（多态的落点）。
 * 装配后 me->stBase.ops 指向某张 const ops 表，之后所有操作经 ops 分发。 */
void uartRing_init(stUartRing *me, const stUartRingConfig *cfg)
{
    me->pUartHandle       = cfg->pUartHandle;
    me->pucRxbuffer       = cfg->pucRxbuffer;
    me->usRxbuffer_size   = cfg->usRxbuffer_size;

    vRingBufInit(&me->stdRingbuffer, cfg->ulRingBufferLen, cfg->pucRingBuffer);

    if (cfg->pTxDmabuffer)
    {
        me->TxDmabuffer_puc = cfg->pTxDmabuffer;
        uart_Ring_tx_initQuene(me);

        // 根据配置选择 ops 表
        if (cfg->ucRxIsIdle)
            me->stBase.ops = &usart_ring_tx_dma_rx_it;  // DMA 空闲中断模式（screen_uart 专用）
        else if (cfg->ucTxIsDMA)
            me->stBase.ops = cfg->ucRxIsIT ? &usart_ring_dma_it_ops : &usart_ring_dma_ops;
        else
            me->stBase.ops = cfg->ucRxIsIT ? &usart_ring_it_ops : &usart_ring_dma_rx_only_ops;
    }
    else
    {
        // 无 TX buffer — 异常路径，不应出现
        me->stBase.ops = cfg->ucRxIsIT ? &usart_ring_it_ops : &usart_ring_dma_rx_only_ops;
    }
}
    
void uart_init_it(stUartRing *me, const stUartRingConfig *cfg)
{
    me->pUartHandle       = cfg->pUartHandle;
    me->pucRxbuffer       = cfg->pucRxbuffer;
    me->usRxbuffer_size   = cfg->usRxbuffer_size;

    vRingBufInit(&me->stdRingbuffer, cfg->ulRingBufferLen, cfg->pucRingBuffer);


    me->TxDmabuffer_puc = cfg->pTxDmabuffer;
    uart_Ring_tx_initQuene(me);

    me->stBase.ops = &usart_ring_tx_dma_rx_it;
    
}



