/*
 * lpuart.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "gpio_reg.h"
#include "lpuart.h"
#include "lpuart_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** USART local macros ***/

// Baud rate.
#define LPUART_BAUD_RATE 		9600
// Buffer sizes.
#define LPUART_TX_BUFFER_SIZE	32
#define LPUART_RX_BUFFER_SIZE	32

/*** LPUART local structures ***/

typedef struct {
	unsigned char tx_buf[LPUART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
	unsigned char rx_buf[LPUART_RX_BUFFER_SIZE]; 	// Receive buffer.
	unsigned int rx_buf_idx;						// Current index in RX buffer.
} LPUART_Context;

/*** LPUART local global variables ***/

static LPUART_Context lpuart_ctx;

/*** LPUART functions ***/

/* CONFIGURE LOW POWER UART.
 * @param:	None.
 * @return:	None.
 */
void LPUART_Init(void) {

	/* Initialize context */
	unsigned int idx = 0;
	for (idx=0 ; idx<LPUART_TX_BUFFER_SIZE ; idx++) lpuart_ctx.tx_buf[idx] = 0;
	lpuart_ctx.tx_buf_write_idx = 0;
	lpuart_ctx.tx_buf_read_idx = 0;
	for (idx=0 ; idx<LPUART_RX_BUFFER_SIZE ; idx++) lpuart_ctx.rx_buf[idx] = 0;
	lpuart_ctx.rx_buf_idx = 0;

	/* Enable peripheral clock*/
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.

	/* Configure TX and RX GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b1111 << 4); // Reset bits 4-7.
	GPIOA -> MODER |= (0b1010 << 4); // Configure PA2 and PA3 as alternate function.
	GPIOA -> AFRL &= 0xFFFF00FF; // Reset bits 8-15.
	GPIOA -> AFRL |= 0x00006600; // Link PA2 and PA3 to AF6.

	/* Configure peripheral */
	LPUART1 -> CR1 = 0; // Disable peripheral while configuring (UE='0'), 1 stop bit and 8 data bits (M = '00').
	LPUART1 -> CR2 = 0; // 1 stop bit (STOP='00').
	LPUART1 -> CR3 = 0;
	LPUART1 -> BRR = ((SYSCLK_MHZ*1000000)/(LPUART_BAUD_RATE))*256; // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	LPUART1 -> CR1 |= (0b11 << 2); // Enable transmitter (TE='1') and receiver (RE='1').

	/* Configure interrupts */
	LPUART1 -> CR1 |= (0b1 << 5); // Enable RX interrupt (RXNEIE='1').
	LPUART1 -> CR1 &= ~(0b1 << 7); // Disable TX interrupt (TXEIE='0'). Enabled when calling LPUART_SendByte() function.
	NVIC_EnableInterrupt(IT_LPUART1);

	/* Enable peripheral */
	LPUART1 -> CR1 |= (0b1 << 0);
}

/* SEND A BYTE THROUGH LOW POWER UART.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void LPUART_SendByte(unsigned char byte_to_send) {

	/* Fill TX buffer with new byte */
	lpuart_ctx.tx_buf[lpuart_ctx.tx_buf_write_idx] = byte_to_send;
	lpuart_ctx.tx_buf_write_idx++;
	if (lpuart_ctx.tx_buf_write_idx == LPUART_RX_BUFFER_SIZE) {
		lpuart_ctx.tx_buf_write_idx = 0;
	}

	/* Enable TXE interrupt */
	LPUART1 -> CR1 |= (0b1 << 7); // TXEIE = '1'.
}

/* SEND A BYTE ARRAY THROUGH LOW POWER UART.
 * @param string_to_send:	Byte array to send.
 * @return:					None.
 */
void LPUART_SendString(char* string_to_send) {

	/* Fill TX buffer with new bytes */
	while (*string_to_send) {
		LPUART_SendByte(*(string_to_send++));
	}

	/* Enable TXE interrupt */
	LPUART1 -> CR1 |= (0b1 << 7); // TXEIE = '1'.
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
			LPUART1 -> CR1 &= ~(0b1 << 7); // No more bytes, disable TXE interrupt (TXEIE = '0').
		}
	}

	/* RXNE interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
		lpuart_ctx.rx_buf[lpuart_ctx.rx_buf_idx] = LPUART1 -> RDR; // Read receive data register and fill incoming byte in RX buffer.
		lpuart_ctx.rx_buf_idx++; // Increment RX index.
		if (lpuart_ctx.rx_buf_idx == LPUART_RX_BUFFER_SIZE) {
			lpuart_ctx.rx_buf_idx = 0; // Manage roll-over.
		}
	}
}
