/*
 * lpuart.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "lpuart.h"

#include "gpio_reg.h"
#include "lpuart_reg.h"
#include "neom8n.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPUART local macros ***/

#define LPUART_BAUD_RATE 		9600 // Baud rate.
#define LPUART_TX_BUFFER_SIZE	128 // TX buffer size.

/*** LPUART local structures ***/

typedef struct {
	unsigned char tx_buf[LPUART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
} LPUART_Context;

/*** LPUART local global variables ***/

static LPUART_Context lpuart_ctx;

/*** LPUART functions ***/

/* CONFIGURE LOW POWER UART.
 * @param:	None.
 * @return:	None.
 */
void LPUART_Init(void) {

	/* Init context */
	unsigned int idx = 0;
	for (idx=0 ; idx<LPUART_TX_BUFFER_SIZE ; idx++) lpuart_ctx.tx_buf[idx] = 0;
	lpuart_ctx.tx_buf_write_idx = 0;
	lpuart_ctx.tx_buf_read_idx = 0;

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.

	/* Configure TX and RX GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b1111 << 26); // Reset bits 26-27.
	GPIOA -> MODER |= (0b1010 << 26); // Configure PA13 and PA14 as alternate function.
	GPIOA -> AFRH &= 0xF00FFFFF; // Reset bits 20-27.
	GPIOA -> AFRH |= 0x06600000; // Link PA13 and PA14 to AF6.

	/* Configure peripheral */
	LPUART1 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	LPUART1 -> CR2 = 0; // 1 stop bit (STOP='00').
	LPUART1 -> CR3 = 0;
	LPUART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='0').
	LPUART1 -> BRR = ((SYSCLK_KHZ*1000)/(LPUART_BAUD_RATE))*256; // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.

	/* Configure DMA and CM interrupt */
	LPUART1 -> CR2 &= (0xFF << 24); // Reset bits 24-31.
	LPUART1 -> CR2 |= (NMEA_LF << 24); // LF character used to trigger CM interrupt.
	LPUART1 -> CR3 |= (0b1 << 6); // Enable DMA transfer for reception (DMAR='1'). Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).

	/* Enable transmitter and interrupts */
	LPUART1 -> CR1 |= (0b1 << 3); // Enable transmitter (TE='1').
	LPUART1 -> CR1 |= (0b1 << 7); // Enable TXE interrupt (TXEIE='1').
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').

	/* Enable interrupt */
	NVIC_EnableInterrupt(IT_LPUART1);

	/* Enable peripheral */
	LPUART1 -> CR1 |= (0b1 << 0);
}

/* ENABLE LPUART RECEPTION.
 * @param:	None.
 * @return:	None.
 */
void LPUART_StartRx(void) {
	// Enable receiver.
	LPUART1 -> CR1 |= (0b1 << 2); // RE='1'.
}

/* DISABLE LPUART RECEPTION.
 * @param:	None.
 * @return:	None.
 */
void LPUART_StopRx(void) {
	// Disable receiver.
	LPUART1 -> CR1 &= ~(0b1 << 2); // RE='0'.
}

/* DISABLE LOW POWER UART.
 * @param:	None.
 * @return:	None.
 */
void LPUART_Off(void) {

	/* Disable interrupts */
	NVIC_DisableInterrupt(IT_LPUART1);

	/* Switch peripheral off */
	LPUART1 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0').
	LPUART1 -> CR1 &= ~(0b1 << 0); // Disable peripheral.

	/* Disable peripheral clock */
	RCC -> APB1ENR &= ~(0b1 << 18); // LPUARTEN='0'.

	/* Put GPIOs in reset state */
	GPIOA -> MODER &= ~(0b1111 << 26); // Configure PA13 and PA14 as input.
}

/* SEND A BYTE THROUGH LOW POWER UART.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void LPUART_SendByte(unsigned char byte_to_send) {

	/* Fill TX buffer with new byte */
	lpuart_ctx.tx_buf[lpuart_ctx.tx_buf_write_idx] = byte_to_send;
	lpuart_ctx.tx_buf_write_idx++;
	if (lpuart_ctx.tx_buf_write_idx == LPUART_TX_BUFFER_SIZE) {
		lpuart_ctx.tx_buf_write_idx = 0;
	}

	/* Enable TXE interrupt */
	LPUART1 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
}

/* LPUART1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_IRQHandler(void) {

	/* TXE interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 7)) != 0) {
		if ((lpuart_ctx.tx_buf_read_idx) != (lpuart_ctx.tx_buf_write_idx)) {
			LPUART1 -> TDR = (lpuart_ctx.tx_buf)[lpuart_ctx.tx_buf_read_idx]; // Fill transmit data register with new byte.
			lpuart_ctx.tx_buf_read_idx++; // Increment TX read index.
			if (lpuart_ctx.tx_buf_read_idx == LPUART_TX_BUFFER_SIZE) {
				lpuart_ctx.tx_buf_read_idx = 0; // Manage roll-over.
			}
		}
		else {
			// No more bytes, disable TXE interrupt.
			LPUART1 -> CR1 &= ~(0b1 << 7); // TXEIE='0'.
		}
	}

	/* Character match interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 17)) != 0) {
		// Tell NEO-M8N driver to switch DMA buffer address.
		NEOM8N_SwitchDmaBuffer();
		// Clear CM flag.
		LPUART1 -> ICR |= (0b1 << 17);
	}

	/* Overrun error interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		LPUART1 -> ICR |= (0b1 << 3);
	}
}
