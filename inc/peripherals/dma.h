/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#ifndef DMA_H
#define DMA_H

/*** DMA functions ***/

void DMA1_Init(void);
void DMA1_Enable(void);
void DMA1_Disable(void);
void DMA1_Start(void);
void DMA1_Stop(void);
void DMA1_SetDestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size);


#endif /* DMA_H */
