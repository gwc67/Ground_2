#include "ring_buffer.h"
#include <string.h>

void vRingBufInit(pstRingBufTdf pstRingBuf,uint32_t ulLen,uint8_t *pucBuf)
{
    pstRingBuf->ulWriteIdx = 0;
    pstRingBuf->ulReadIdx = 0;

    pstRingBuf->ulLength = ulLen;
    pstRingBuf->pucBuffer = pucBuf;
    pstRingBuf->ulItemSize = 1;

    for (int i = 0; i < ulLen; i++)
    {
        pucBuf[i] = 0;
    }
}

uint8_t ucRingBufWrite(pstRingBufTdf pstRingBuf,uint8_t ucValue)
{
    uint32_t ulNextWrite = pstRingBuf->ulWriteIdx + 1;
    if (ulNextWrite == pstRingBuf->ulLength)
    {
        ulNextWrite = 0;
    }

    if (ulNextWrite == pstRingBuf->ulReadIdx)
    {
        return 1;
    }
    else
    {
        pstRingBuf->pucBuffer[pstRingBuf->ulWriteIdx] = ucValue;
        pstRingBuf->ulWriteIdx = ulNextWrite;
    }
    return 0;
}


uint8_t ucRingBufRead(pstRingBufTdf pstRingBuf,uint8_t* pucData)
{
    if (pstRingBuf->ulReadIdx == pstRingBuf->ulWriteIdx)
    {
        return 1;
    }
    else
    {
        *pucData = pstRingBuf->pucBuffer[pstRingBuf->ulReadIdx];
        pstRingBuf->ulReadIdx++;
        if (pstRingBuf->ulReadIdx == pstRingBuf->ulLength)
        {
            pstRingBuf->ulReadIdx = 0;
        }
    }
    return 0;
}

//类似获取数据长度的函数
uint8_t ucRingBufGetLength(pstRingBufTdf pstRingBuf)
{
    return (pstRingBuf->ulWriteIdx - pstRingBuf->ulReadIdx + pstRingBuf->ulLength) % pstRingBuf->ulLength;
}

//获取接受数据的第几个数据
uint8_t ucRingBufPeek(pstRingBufTdf pstRingBuf,uint32_t ulPosition)
{
    if (pstRingBuf->ulLength == pstRingBuf->ulReadIdx)
    {
        return 0xff;
    }

    uint32_t ulRealPos = (pstRingBuf->ulLength + ulPosition) % pstRingBuf->ulLength;
    return pstRingBuf->pucBuffer[ulRealPos];
}

void vRingBufItemInit(pstRingBufTdf pstRingBuf, uint32_t ulItemCapacity, uint32_t ulItemSize, uint8_t *pucBuf)
{
    pstRingBuf->ulWriteIdx = 0;
    pstRingBuf->ulReadIdx = 0;
    pstRingBuf->ulLength = ulItemCapacity;
    pstRingBuf->ulItemSize = ulItemSize;
    pstRingBuf->pucBuffer = pucBuf;
}

uint8_t ucRingBufWriteItem(pstRingBufTdf pstRingBuf, const void *pvData)
{
    uint32_t ulNextWrite = pstRingBuf->ulWriteIdx + 1;
    if (ulNextWrite >= pstRingBuf->ulLength)
    {
        ulNextWrite = 0;
    }

    if (ulNextWrite == pstRingBuf->ulReadIdx)
    {
        return 1;
    }
    else
    {
        uint32_t ulByteOffset = pstRingBuf->ulWriteIdx * pstRingBuf->ulItemSize;
        memcpy((void*)&pstRingBuf->pucBuffer[ulByteOffset], pvData, pstRingBuf->ulItemSize);
        pstRingBuf->ulWriteIdx = ulNextWrite;
    }
    return 0;
}

uint8_t ucRingBufReadItem(pstRingBufTdf pstRingBuf, void *pvData)
{
    if (pstRingBuf->ulReadIdx == pstRingBuf->ulWriteIdx)
    {
        return 1;
    }
    else
    {
        uint32_t ulByteOffset = pstRingBuf->ulReadIdx * pstRingBuf->ulItemSize;
        memcpy(pvData, &pstRingBuf->pucBuffer[ulByteOffset], pstRingBuf->ulItemSize);
        pstRingBuf->ulReadIdx++;
        if (pstRingBuf->ulReadIdx >= pstRingBuf->ulLength)
        {
            pstRingBuf->ulReadIdx = 0;
        }
    }
    return 0;
}

uint8_t ucRingBufGetItemCount(pstRingBufTdf pstRingBuf)
{
    return (pstRingBuf->ulWriteIdx - pstRingBuf->ulReadIdx + pstRingBuf->ulLength) % pstRingBuf->ulLength;
}

uint8_t ucRingBufIsFull(pstRingBufTdf pstRingBuf)
{
    uint32_t ulNextWrite = pstRingBuf->ulWriteIdx + 1;
    if (ulNextWrite >= pstRingBuf->ulLength)
    {
        ulNextWrite = 0;
    }
    return (ulNextWrite == pstRingBuf->ulReadIdx) ? 1 : 0;
}

uint8_t ucRingBufIsEmpty(pstRingBufTdf pstRingBuf)
{
    return (pstRingBuf->ulReadIdx == pstRingBuf->ulWriteIdx) ? 1 : 0;
}

void ucRingBufClear(pstRingBufTdf pstRingBuf)
{
    pstRingBuf->ulReadIdx = 0;
    pstRingBuf->ulWriteIdx = 0;
}
