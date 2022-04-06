/*
 * dma.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#ifndef DMA_H
#define DMA_H

/*** DMA functions ***/

void DMA1_init_channel6(void);
void DMA1_start_channel6(void);
void DMA1_stop_channel6(void);
void DMA1_set_channel6_dest_addr(unsigned int dest_buf_addr, unsigned short dest_buf_size);

#endif /* DMA_H */
