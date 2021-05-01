/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#ifndef DMA_H
#define DMA_H

/*** DMA functions ***/

void DMA1_InitChannel6(void);
void DMA1_StartChannel6(void);
void DMA1_StopChannel6(void);
void DMA1_SetChannel6DestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size);
void DMA1_Disable(void);

#endif /* DMA_H */
