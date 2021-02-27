/*
 * dma.c
 *
 *  Created on: 8 may 2018
 *      Author: Ludo
 */

#include "dma.h"

#include "dma_reg.h"
#include "lpuart_reg.h"
#include "neom8n.h"
#include "nvic.h"
#include "rcc_reg.h"

/*** DMA local macros ***/

#define DMA_DEFAULT_BUFFER_SIZE		128

/*** DMA local global variables ***/

static unsigned char dma_default_buffer[DMA_DEFAULT_BUFFER_SIZE];

/*** DMA local functions ***/

/* DMA1 CHANNEL 3 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Channel2_3_IRQHandler(void) {
	// Transfer complete interrupt (TCIF3='1').
	if (((DMA1 -> ISR) & (0b1 << 9)) != 0) {
		// Switch DMA buffer without decoding.
		NEOM8N_SwitchDmaBuffer(0);
		// Clear flag.
		DMA1 -> IFCR |= (0b1 << 9); // CTCIF3='1'.
	}
}

/*** DMA functions ***/

/* CONFIGURE DMA1 CHANNEL3 FOR LPUART RX BYTES TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Init(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
	// Configure peripheral.
	DMA1 -> CPAR3 = (unsigned int) &(LPUART1 -> RDR); // Peripheral address = LPUART RX register.
	DMA1 -> CMAR3 = (unsigned int) &(dma_default_buffer);
	DMA1 -> CNDTR3 = DMA_DEFAULT_BUFFER_SIZE;
	// Disable DMA channel before configuration (EN='0').
	// Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	// Disable memory to memory mode (MEM2MEM='0').
	// Peripheral increment mode disabled (PINC='0').
	// Circular mode disabled (CIRC='0').
	// Read from peripheral (DIR='0').
	DMA1 -> CCR3 |= (0b11 << 12); // Very high priority (PL='11').
	DMA1 -> CCR3 |= (0b1 << 7); // Memory increment mode enabled (MINC='1').
	DMA1 -> CCR3 |= (0b1 << 1); // Enable transfer complete interrupt (TCIE='1').
	// Configure channel 3 for LPUART RX (request number 5).
	DMA1 -> CSELR &= ~(0b1111 << 8); // Reset bits 8-11.
	DMA1 -> CSELR |= (0b0101 << 8); // DMA channel mapped on LPUART1_RX (C3S='0101').
}

/* DISABLE DMA1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Disable(void) {
	// Disable DMA1 channel 3.
	NVIC_DisableInterrupt(NVIC_IT_DMA1_CH_2_3);
	RCC -> AHBENR &= ~(0b1 << 0); // DMAEN='0'.
}

/* START DMA1 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Start(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x0FFFFFFF;
	NVIC_EnableInterrupt(NVIC_IT_DMA1_CH_2_3);
	// Start transfer.
	DMA1 -> CCR3 |= (0b1 << 0); // EN='1'.
}

/* stop DMA1 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_Stop(void) {
	// Stop transfer.
	DMA1 -> CCR3 &= ~(0b1 << 0); // EN='0'.
	NVIC_DisableInterrupt(NVIC_IT_DMA1_CH_2_3);
}

/* SET DMA1 DESTINATION BUFFER ADDRESS FOR STORING LPUART RX BYTES.
 * @param dest_buf_addr:	Address of destination buffer (GPS frame).
 * @param dest_buf_size:	Size of destination buffer (16 bits word).
 * @return:					None.
 */
void DMA1_SetDestAddr(unsigned int dest_buf_addr, unsigned short dest_buf_size) {
	// Set address.
	DMA1 -> CMAR3 = dest_buf_addr;
	// Set buffer size.
	DMA1 -> CNDTR3 = dest_buf_size;
	// Clear all flags.
	DMA1 -> IFCR |= 0x0FFFFFFF;
}
