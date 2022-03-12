/*
 * usart.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#include "usart.h"

#include "at.h"
#include "gpio.h"
#include "lptim.h"
#include "mode.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "usart_reg.h"

/*** USART local macros ***/

#ifdef ATM
#define USART_BAUD_RATE 			9600
#define USART_TIMEOUT_COUNT			100000
#define USART_STRING_LENGTH_MAX		1000
#endif

/*** USART local functions ***/

#if (defined ATM) && (defined HW1_0)
/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
	// RXNE interrupt.
	if (((USART2 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_fill_rx_buffer(USART2 -> RDR);
		// Clear RXNE flag.
		USART2 -> RQR |= (0b1 << 3);
	}
	// Overrun error interrupt.
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
}
#endif

#if (defined ATM) && (defined HW2_0)
/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) USART1_IRQHandler(void) {
	// RXNE interrupt.
	if (((USART1 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_fill_rx_buffer(USART1 -> RDR);
	}
	// Overrun error interrupt.
	if (((USART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART1 -> ICR |= (0b1 << 3);
	}
}
#endif

#ifdef ATM
/* FILL USART TX BUFFER WITH A NEW BYTE.
 * @param tx_byte:	Byte to append.
 * @return status:	Function execution status.
 */
static USART_status_t USARTx_FillTxBuffer(unsigned char tx_byte) {
	// Local variables.
	USART_status_t status = USART_SUCCESS;
	unsigned int loop_count = 0;
	// Fill transmit register.
#ifdef HW1_0
	USART2 -> TDR = tx_byte;
#endif
#ifdef HW2_0
	USART1 -> TDR = tx_byte;
#endif
	// Wait for transmission to complete.
#ifdef HW1_0
	while (((USART2 -> ISR) & (0b1 << 7)) == 0) {
#endif
#ifdef HW2_0
	while (((USART1 -> ISR) & (0b1 << 7)) == 0) {
#endif
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > USART_TIMEOUT_COUNT) {
			status = USART_ERROR_TX_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}
#endif

/*** USART functions ***/

#ifdef HW1_0
/* CONFIGURE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_init(void) {
#ifdef ATM
	// Enable peripheral clock.
	RCC -> CR |= (0b1 << 1); // Enable HSI in stop mode (HSI16KERON='1').
	RCC -> CCIPR |= (0b10 << 2); // Select HSI as USART clock.
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.
	RCC -> APB1SMENR |= (0b1 << 17); // Enable clock in sleep mode.
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_USART2_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART2_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Configure peripheral.
	USART2 -> CR3 |= (0b1 << 12) | (0b1 << 23); // No overrun detection (OVRDIS='1') and clock enable in stop mode (UCESM='1').
	USART2 -> BRR = ((RCC_HSI_FREQUENCY_KHZ * 1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Enable transmitter and receiver.
	USART2 -> CR1 |= (0b1 << 5) | (0b11 << 2); // TE='1', RE='1' and RXNEIE='1'.
	// Set interrupt priority.
	NVIC_set_priority(NVIC_IT_USART2, 3);
	// Enable peripheral.
	USART2 -> CR1 |= (0b11 << 0);
#else
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_USART2_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART2_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}
#endif

#ifdef HW2_0
/* CONFIGURE USART1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART1_init(void) {
#ifdef ATM
	// Enable peripheral clock.
	RCC -> CR |= (0b1 << 1); // Enable HSI in stop mode (HSI16KERON='1').
	RCC -> CCIPR |= (0b10 << 0); // Select HSI as USART clock.
	RCC -> APB2ENR |= (0b1 << 14); // USART1EN='1'.
	RCC -> APB2SMENR |= (0b1 << 14); // Enable clock in sleep mode.
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_USART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Configure peripheral.
	USART1 -> CR3 |= (0b1 << 12) | (0b1 << 23); // No overrun detection (OVRDIS='1') and clock enable in stop mode (UCESM='1').
	USART1 -> BRR = ((RCC_HSI_FREQUENCY_KHZ * 1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Enable transmitter and receiver.
	USART1 -> CR1 |= (0b1 << 5) | (0b11 << 2); // TE='1', RE='1' and RXNEIE='1'.
	// Set interrupt priority.
	NVIC_set_priority(NVIC_IT_USART1, 3);
	// Enable peripheral.
	USART1 -> CR1 |= (0b11 << 0);
#else
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_USART1_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART1_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}
#endif

#ifdef ATM
/* SEND A BYTE ARRAY THROUGH USART2.
 * @param tx_string:	Byte array to send.
 * @return status:		Function execution status.
 */
USART_status_t USARTx_send_string(char* tx_string) {
	// Local variables.
	USART_status_t status = USART_SUCCESS;
	unsigned int char_count = 0;
	// Disable interrupt.
#ifdef HW1_0
	NVIC_disable_interrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_disable_interrupt(NVIC_IT_USART1);
#endif
	// Loop on all characters.
	while (*tx_string) {
		// Fill TX buffer with new byte.
		status = USARTx_FillTxBuffer((unsigned char) *(tx_string++));
		if (status != USART_SUCCESS) break;
		// Check char count.
		char_count++;
		if (char_count > USART_STRING_LENGTH_MAX) {
			status = USART_ERROR_STRING_LENGTH;
			break;
		}
	}
	// Enable interrupt.
#ifdef HW1_0
	NVIC_enable_interrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_enable_interrupt(NVIC_IT_USART1);
#endif
	return status;
}
#endif
