#ifndef __ANO_H
#define __ANO_H

#include "ano_base.h"

/* 地面站 ANO 设备 */
extern stAnoBase* pstAnobase_Ground;    /* USART1 → 飞控（DMA） */
extern stAnoBase* pstAnobase_Usart3;    /* USART3 → IT TX + IT RX */

void ano_board_init(void);

#endif
