#ifndef __ANO_TURE_H
#define __ANO_TURE_H

#include "ano_base.h"

typedef struct
{
    // void (*Ano_init)(void);
    void (*Ano_Receive_Anl)(uint8_t* pucData,uint8_t uclen);
    void (*Ano_Add_Send_Data)(uint8_t ucFrame_num,uint8_t *pCnt,uint8_t* pucData);
    void (*Send_Buff)(uint8_t *pucData,uint8_t ucLength);
}unique;
//对于不同的USART，其必定拥有不同的初始化，不同的初始化解析函数，我认为需要传入该特殊化的指针

typedef struct 
{
    stAnoBase stBase;
    _dt_st* pstDeviceDt;
    u8* pucTxBuffer;
    u8* pucRxBuffer;
    unique stUser; // 特殊化的函数指针
    uint8_t ucData_Len;
    uint8_t ucData_Cnt;
    uint8_t ucRxstate;
}stAnoDevice;

void Ano_Device_init(stAnoDevice* me,
                    _dt_st* pstDeviceDt,
                    uint8_t* pucTxBuffer,
                    uint8_t* pucRxBuffer,
                    unique* pstUser);

#endif
