#ifndef __ANO_DEVICE_USART3_H
#define __ANO_DEVICE_USART3_H

#include "main.h"

/* ============== USART3 ANO 设备层 — IT TX + IT RX ============== */


void vUsart3_init_Ano(void);

void vUsart3_DT_Data_Receive_Anl_Ano(uint8_t *pucdata, uint8_t uclen);
void vUsart3_Add_Send_Data_Ano(uint8_t ucFrame_num, uint8_t *pcnt, uint8_t *pucTxBuffer);
void vUsart3_TxBuffer_Ano(uint8_t *pucData, uint8_t ucLength);
void vUsart3_Data_Exchange_Task_Ano(void);

#endif
