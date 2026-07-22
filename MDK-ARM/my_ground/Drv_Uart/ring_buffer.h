#ifndef __RING_BUF_H
#define __RING_BUF_H

#include "main.h"
typedef struct
{
    volatile uint32_t ulReadIdx;
    volatile uint32_t ulWriteIdx;
    volatile uint32_t ulLength;
    uint8_t  *pucBuffer;
    uint32_t ulItemSize;
}stRingBufTdf,*pstRingBufTdf;

void vRingBufInit(pstRingBufTdf pstRingBuf,uint32_t ulLen , uint8_t *pucBuf);

uint8_t ucRingBufWrite(pstRingBufTdf pstRingBuf,uint8_t ucValue);

uint8_t ucRingBufRead(pstRingBufTdf pstRingBuf,uint8_t *pucData);

uint8_t ucRingBufGetLength(pstRingBufTdf pstRingBuf);

uint8_t ucRingBufPeek(pstRingBufTdf psRingBuf, uint32_t ulPostion);

void vRingBufItemInit(pstRingBufTdf pstRingBuf, uint32_t ulItemCapacity, uint32_t ulItemSize, uint8_t *pucBuf);
uint8_t ucRingBufWriteItem(pstRingBufTdf pstRingBuf, const void *pvData);
uint8_t ucRingBufReadItem(pstRingBufTdf pstRingBuf, void *pvData);
uint8_t ucRingBufGetItemCount(pstRingBufTdf pstRingBuf);
uint8_t ucRingBufIsFull(pstRingBufTdf pstRingBuf);
uint8_t ucRingBufIsEmpty(pstRingBufTdf pstRingBuf);
void ucRingBufClear(pstRingBufTdf pstRingBuf);

#endif  
