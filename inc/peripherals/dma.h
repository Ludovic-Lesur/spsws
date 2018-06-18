/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_DMA_H
#define PERIPHERALS_DMA_H

/*** DMA functions ***/

void DMA_LpuartRxInit(void);
void DMA_LpuartRxOff(void);
void DMA_LpuartRxSetDestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size);
void DMA_LpuartRxStart(void);
void DMA_LpuartRxStop(void);

#endif /* PERIPHERALS_DMA_H */
