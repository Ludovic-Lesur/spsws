/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_DMA_H
#define PERIPHERALS_DMA_H

/*** DMA functions ***/

void DMA_Init(void);
void DMA_Off(void);
void DMA_SetDestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size);
void DMA_Start(void);
void DMA_Stop(void);

#endif /* PERIPHERALS_DMA_H */
