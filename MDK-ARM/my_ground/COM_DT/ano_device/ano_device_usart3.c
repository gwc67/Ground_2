/**
 * ano_device_usart3.c — USART3 ANO 协议设备层（IT TX + IT RX）
 *
 * 物理层：uart_transmit(pstbase_usart3_uart, ...) — IT 模式，
 *         队列非阻塞发送（见 uart_Ring_tx_IT）。
 *
 * 按需修改 vUsart3_DT_Data_Receive_Anl_Ano 中的 CMD 分发，
 * 以及 vUsart3_Add_Send_Data_Ano 中的发送帧填充。
 */
#include "ano_device_usart3.h"
#include "ano.h"
#include "uarts.h"
#include "driver_registry.h"
#include "string.h"
#include "ano_device_ground.h"
#include "SysConfig.h"

/* ============== 初始化 ============== */
//电池
#define LX_BAT 0x0D

void vUsart3_init_Ano(void)
{
    vano_sendID_set(pstAnobase_Usart3, 0x00, 0);
    vano_sendID_set(pstAnobase_Usart3, LX_BAT, 20);
}
DRIVER_INIT(vUsart3_init_Ano);

/* ============== 接收帧解析 ============== */

void vUsart3_DT_Data_Receive_Anl_Ano(uint8_t *pucdata, uint8_t uclen)
{
    u8 check_sum1 = 0, check_sum2 = 0;

    if (*(pucdata + 3) != (uclen - 6))
        return;
    for (u8 i = 0; i < uclen - 2; i++)
    {
        check_sum1 += *(pucdata + i);
        check_sum2 += check_sum1;
    }
    if ((check_sum1 != *(pucdata + uclen - 2)) || (check_sum2 != *(pucdata + uclen - 1)))
        return;
    if (*(pucdata) != 0xAA || *(pucdata + 1) != 0xFF)
        return;

    if (*(pucdata + 2) == 0XE0)
    {
        /* 命令帧：按需分发 */
        switch (*(pucdata + 4))
        {
        case 0x01: break;
        case 0x02: break;
        default:   break;
        }
        ano_ck_back_v(pstAnobase_Usart3, 0xff, *(pucdata + 2), check_sum1, check_sum2);
    }
    else if (*(pucdata + 2) == 0X00)
    {
        /* 校验确认帧 */
        if (ano_ck_id_get(pstAnobase_Usart3) == *(pucdata + 4) &&
            ano_ck_sc_get(pstAnobase_Usart3) == *(pucdata + 5) &&
            ano_ck_ac_get(pstAnobase_Usart3) == *(pucdata + 6))
        {
            vano_wait_ck_clear(pstAnobase_Usart3);
        }
    }
    else if (*(pucdata + 2) == 0XE1)
    {
        uint16_t par_id = *(pucdata + 4) + *(pucdata + 5) * 256;
        ano_par_back_v(pstAnobase_Usart3, 0xff, par_id, 0);
    }
    else if (*(pucdata + 2) == 0xE2)
    {
        ano_ck_back_v(pstAnobase_Usart3, 0xff, *(pucdata + 2), check_sum1, check_sum2);
    }
}

/* ============== 发送数据填充 ============== */

void vUsart3_Add_Send_Data_Ano(uint8_t ucFrame_num, uint8_t *pcnt, uint8_t *pucTxBuffer)
{
    switch (ucFrame_num)
    {
    case 0x00: {
        pucTxBuffer[(*pcnt)++] = ano_ck_id_get(pstAnobase_Usart3);
        pucTxBuffer[(*pcnt)++] = ano_ck_sc_get(pstAnobase_Usart3);
        pucTxBuffer[(*pcnt)++] = ano_ck_ac_get(pstAnobase_Usart3);
    }
    break;

    case LX_BAT: {
        struct gs_batt_curr_height_process_t snap;
        gs_batt_curr_height_process_copy(&snap);
        pucTxBuffer[(*pcnt)++]  = BYTE0(snap.voltage_100);
        pucTxBuffer[(*pcnt)++]  = BYTE1(snap.voltage_100);
        pucTxBuffer[(*pcnt)++]  = BYTE0(snap.current_100);
        pucTxBuffer[(*pcnt)++]  = BYTE1(snap.current_100);
    }
    break;

    default:
        break;
    }
}

/* ============== 物理层发送 ============== */

void vUsart3_TxBuffer_Ano(uint8_t *pucData, uint8_t ucLength)
{
    uart_transmit(pstbase_usart3_uart, pucData, ucLength);
}

/* ============== 周期发送调度 ============== */

void vUsart3_Data_Exchange_Task_Ano(void)
{
    vano_ck_back_check(pstAnobase_Usart3);
    vano_check_to_send(pstAnobase_Usart3, 0x00);
    vano_check_to_send(pstAnobase_Usart3, 0xe0);
    vano_check_to_send(pstAnobase_Usart3, LX_BAT);
}
