/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#ifndef __DMA_H__
#define __DMA_H__

#include "types.h"

/*** DMA functions ***/

void DMA1_init_channel6(void);
void DMA1_start_channel6(void);
void DMA1_stop_channel6(void);
void DMA1_set_channel6_dest_addr(uint32_t dest_buf_addr, uint16_t dest_buf_size);

#endif /* __DMA_H__ */
