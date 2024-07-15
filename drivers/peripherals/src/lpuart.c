/*
 * lpuart.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "lpuart.h"

#include "exti.h"
#include "gpio.h"
#include "lpuart_reg.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "string.h"
#include "types.h"

/*** LPUART local macros ***/

#define LPUART_BAUD_RATE 		9600
#define LPUART_TIMEOUT_COUNT	100000

/*** LPUART local global variables ***/

static LPUART_character_match_irq_cb_t lpuart1_cm_irq_callback = NULL;

/*** LPUART local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPUART1_IRQHandler(void) {
	// Character match interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 17)) != 0) {
		// Notify upper layer.
		if ((((LPUART1 -> CR1) & (0b1 << 14)) != 0) && (lpuart1_cm_irq_callback != NULL)) {
			lpuart1_cm_irq_callback();
		}
		// Clear CM flag.
		LPUART1 -> ICR |= (0b1 << 17);
	}
	// Overrun error interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		LPUART1 -> ICR |= (0b1 << 3);
	}
	EXTI_clear_flag(EXTI_LINE_LPUART1);
}

/*******************************************************************/
static LPUART_status_t _LPUART1_fill_tx_buffer(uint8_t tx_byte) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	uint32_t loop_count = 0;
	// Fill transmit register.
	LPUART1 -> TDR = tx_byte;
	// Wait for transmission to complete.
	while (((LPUART1 -> ISR) & (0b1 << 7)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) {
			status = LPUART_ERROR_TX_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}

/*** LPUART functions ***/

/*******************************************************************/
LPUART_status_t LPUART1_init(LPUART_character_match_irq_cb_t irq_callback) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	RCC_clock_t lpuart_clock = RCC_CLOCK_LSE;
	uint8_t lse_status = 0;
	uint32_t lpuart_clock_hz = 0;
	uint64_t brr = 0;
	// Select peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 10); // Reset bits 10-11.
	// Get LSE status.
	rcc_status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	RCC_exit_error(LPUART_ERROR_BASE_RCC);
	// Check LSE status.
	if (lse_status != 0) {
		// Use LSE.
		RCC -> CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
		lpuart_clock = RCC_CLOCK_LSE;
	}
	else {
		// Use HSI.
		RCC -> CCIPR |= (0b10 << 10); // LPUART1SEL='10'.
		lpuart_clock = RCC_CLOCK_HSI;
	}
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(lpuart_clock, &lpuart_clock_hz);
	RCC_exit_error(LPUART_ERROR_BASE_RCC);
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.
	// Configure peripheral.
	LPUART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='0').
	// Baud rate.
	brr = ((uint64_t) lpuart_clock_hz) << 8;
	brr /= (uint64_t) LPUART_BAUD_RATE;
	LPUART1 -> BRR = (uint32_t) (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Configure character match interrupt and DMA.
	LPUART1 -> CR2 |= (STRING_CHAR_LF << 24); // LF character used to trigger CM interrupt.
	LPUART1 -> CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
	// Enable transmitter and receiver.
	LPUART1 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
	// Configure GPIOs.
#if !(defined HW1_0) || !(defined DEBUG)
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Register callback.
	lpuart1_cm_irq_callback = irq_callback;
errors:
	return status;
}

/*******************************************************************/
void LPUART1_de_init(void) {
	// Disable LPUART alternate function.
#if !(defined HW1_0) || !(defined DEBUG)
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Disable peripheral.
	LPUART1 -> CR1 &= ~(0b1 << 0); // UE='0'.
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 18); // LPUARTEN='0'.
}

/*******************************************************************/
LPUART_status_t LPUART1_write(uint8_t* data, uint32_t data_size_bytes) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	uint32_t idx = 0;
	uint32_t loop_count = 0;
	// Check parameter.
	if (data == NULL) {
		status = LPUART_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Fill TX buffer with new bytes.
	for (idx=0 ; idx<data_size_bytes ; idx++) {
		status = _LPUART1_fill_tx_buffer(data[idx]);
		if (status != LPUART_SUCCESS) goto errors;
	}
	// Wait for TC flag.
	while (((LPUART1 -> ISR) & (0b1 << 6)) == 0) {
		// Exit if timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) {
			status = LPUART_ERROR_TC_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}
