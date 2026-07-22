#include "ano_ture.h"
#include "ano_device_ground.h"
#include "ano_device_usart3.h"
#include "driver_registry.h"

/* ============== Ground — USART1 连接飞控 ============== */

stAnoBase* pstAnobase_Ground;

static uint8_t ucTxBuffer_Ground[256];
static uint8_t ucRxBuffer_Ground[256];
static _dt_st  stDeviceDt_Ground;
static stAnoDevice stAnoDevice_Ground;

static unique stUser_Ground = {
    .Ano_Add_Send_Data = vGround_Add_Send_Data_Ano,
    .Ano_Receive_Anl   = vGround_DT_Data_Receive_Anl_Ano,
    .Send_Buff         = vGround_TxBuffer_Ano,
};

/* ============== USART3 — IT TX + IT RX，ANO 协议 ============== */

stAnoBase* pstAnobase_Usart3;

static uint8_t ucTxBuffer_Usart3[50];
static uint8_t ucRxBuffer_Usart3[256];
static _dt_st  stDeviceDt_Usart3;
static stAnoDevice stAnoDevice_Usart3;

static unique stUser_Usart3 = {
    .Ano_Add_Send_Data = vUsart3_Add_Send_Data_Ano,
    .Ano_Receive_Anl   = vUsart3_DT_Data_Receive_Anl_Ano,
    .Send_Buff         = vUsart3_TxBuffer_Ano,
};

void ano_board_init(void)
{
    /* Ground — USART1 */
    Ano_Device_init(&stAnoDevice_Ground,
                    &stDeviceDt_Ground,
                    ucTxBuffer_Ground,
                    ucRxBuffer_Ground,
                    &stUser_Ground);
    pstAnobase_Ground = &stAnoDevice_Ground.stBase;

    /* USART3 — IT TX + IT RX */
    Ano_Device_init(&stAnoDevice_Usart3,
                    &stDeviceDt_Usart3,
                    ucTxBuffer_Usart3,
                    ucRxBuffer_Usart3,
                    &stUser_Usart3);
    pstAnobase_Usart3 = &stAnoDevice_Usart3.stBase;
}

/* ano_board_init 在 uart_board_init 之后执行（b1 < b2） */
DRIVER_INIT_2(ano_board_init);
