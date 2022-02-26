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

/*** DMA local functions ***/

/* DMA1 CHANNEL 6 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) DMA1_Channel4_5_6_7_IRQHandler(void) {
	// Transfer complete interrupt (TCIF6='1').
	if (((DMA1 -> ISR) & (0b1 << 21)) != 0) {
		// Switch DMA buffer without decoding.
		if (((DMA1 -> CCR6) & (0b1 << 1)) != 0) {
			NEOM8N_switch_dma_buffer(0);
		}
		// Clear flag.
		DMA1 -> IFCR |= (0b1 << 21); // CTCIF6='1'.
	}
}

/*** DMA functions ***/

/* CONFIGURE DMA1 CHANNEL 6 FOR LPUART RX TRANSFER (NMEA FRAMES FROM GPS MODULE).
 * @param:	None.
 * @return:	None.
 */
void DMA1_init_channel6(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
	// Disable DMA channel before configuration (EN='0').
	// Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	// Disable memory to memory mode (MEM2MEM='0').
	// Peripheral increment mode disabled (PINC='0').
	// Circular mode disabled (CIRC='0').
	// Read from peripheral (DIR='0').
	DMA1 -> CCR6 |= (0b11 << 12); // Very high priority (PL='11').
	DMA1 -> CCR6 |= (0b1 << 7); // Memory increment mode enabled (MINC='1').
	DMA1 -> CCR6 |= (0b1 << 1); // Enable transfer complete interrupt (TCIE='1').
	DMA1 -> CCR6 &= ~(0b1 << 4); // Read from peripheral.
	// Configure peripheral address.
	DMA1 -> CPAR6 = (unsigned int) &(LPUART1 -> RDR); // Peripheral address = LPUART RX register.
	// Configure channel 3 for LPUART1 RX (request number 5).
	DMA1 -> CSELR &= ~(0b1111 << 20); // Reset bits 20-23.
	DMA1 -> CSELR |= (0b0101 << 20); // DMA channel mapped on LPUART1_RX (C6S='0101').
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
	// Set interrupt priority.
	NVIC_set_priority(NVIC_IT_DMA1_CH_4_7, 1);
}

/* START DMA1 CHANNEL 6 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_start_channel6(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
	NVIC_enable_interrupt(NVIC_IT_DMA1_CH_4_7);
	// Start transfer.
	DMA1 -> CCR6 |= (0b1 << 0); // EN='1'.
}

/* STOP DMA1 CHANNEL 6 TRANSFER.
 * @param:	None.
 * @return:	None.
 */
void DMA1_stop_channel6(void) {
	// Stop transfer.
	DMA1 -> CCR6 &= ~(0b1 << 0); // EN='0'.
	NVIC_disable_interrupt(NVIC_IT_DMA1_CH_4_7);
}

/* SET DMA1 CHANNEL 6 DESTINATION BUFFER ADDRESS.
 * @param dest_buf_addr:	Address of destination buffer (NMEA frame).
 * @param dest_buf_size:	Size of destination buffer.
 * @return:					None.
 */
void DMA1_set_channel6_dest_addr(unsigned int dest_buf_addr, unsigned short dest_buf_size) {
	// Set address.
	DMA1 -> CMAR6 = dest_buf_addr;
	// Set buffer size.
	DMA1 -> CNDTR6 = dest_buf_size;
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
}

/* DISABLE DMA1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void DMA1_disable(void) {
	// Disable interrupts.
	NVIC_disable_interrupt(NVIC_IT_DMA1_CH_2_3);
	NVIC_disable_interrupt(NVIC_IT_DMA1_CH_4_7);
	// Clear all flags.
	DMA1 -> IFCR |= 0x0FFFFFFF;
	// Disable peripheral clock.
	RCC -> AHBENR &= ~(0b1 << 0); // DMAEN='0'.
}
