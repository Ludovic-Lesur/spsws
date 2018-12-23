/*
 * dma.c
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#include "dma.h"

#include "dma_reg.h"
#include "lpuart_reg.h"
#include "rcc_reg.h"

/* CONFIGURE DMA1 CHANNEL3 FOR LPUART RX BYTES TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Init(void) {

	/* Enable peripheral clock */
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.

	/* Configure channel 3 for LPUART RX (request number 5) */
	DMA1 -> CCR3 &= ~(0b1 << 0); // Disable DMA channel before configuration (EN='0').
	DMA1 -> CCR3 &= ~(0b1 << 14); // Disable memory to memory mode (MEM2MEM='0').
	DMA1 -> CCR3 |= (0b11 << 12); // Very high priority (PL='11').
	DMA1 -> CCR3 &= ~(0b1111 << 8); // Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	DMA1 -> CCR3 |= (0b1 << 7); // Memory increment mode enabled (MINC='1').
	DMA1 -> CCR3 &= ~(0b1 << 6); // Peripheral increment mode disabled (PINC='0').
	DMA1 -> CCR3 &= ~(0b1 << 5); // Circular mode disabled (CIRC='0').
	DMA1 -> CCR3 &= ~(0b1 << 4); // Read from peripheral (DIR='0').
	DMA1 -> CPAR3 = (unsigned int) &(LPUART1 -> RDR); // Peripheral address = LPUART read data register.
	DMA1 -> CSELR &= ~(0b1111 << 8); // Reset bits 8-11.
	DMA1 -> CSELR |= (0b0101 << 8); // DMA channel mapped on LPUART1_RX (C3S='0101').

	/* Disable peripheral by default */
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='0'.
}

/* ENABLE DMA1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Enable(void) {

	/* Enable DMA1 channel 3 */
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
}

/* DISABLE DMA1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Disable(void) {

	/* Disable DMA1 channel 3 */
	RCC -> AHBENR &= ~(0b1 << 0); // DMAEN='0'.
}

/* START DMA1 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Start(void) {

	/* Start transfer */
	DMA1 -> CCR3 |= (0b1 << 0); // EN='1'.
}

/* stop DMA1 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Stop(void) {

	/* Stop transfer */
	DMA1 -> CCR3 &= ~(0b1 << 0); // EN='0'.
}

/* SET DMA1 DESTINATION BUFFER ADDRESS FOR STORING LPUART RX BYTES.
 * @param dest_buf_addr:	Address of destination buffer (GPS frame).
 * @param dest_buf_size:	Size of destination buffer (16 bits word).
 * @return:					None.
 */
void DMA1_SetDestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size) {
	DMA1 -> CMAR3 = dest_buf_addr;
	DMA1 -> CNDTR3 = dest_buf_size;
}
