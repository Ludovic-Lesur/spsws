/*
 * lpuart.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "lpuart.h"

#include "gpio.h"
#include "lptim.h"
#include "lpuart_reg.h"
#include "mapping.h"
#include "neom8n.h"
#include "nvic.h"
#include "string.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPUART local macros ***/

#define LPUART_BAUD_RATE 		9600
#define LPUART_TIMEOUT_COUNT	100000

/*** LPUART local functions ***/

/* LPUART1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) LPUART1_IRQHandler(void) {
	// Character match interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 17)) != 0) {
		// Switch DMA buffer and decode buffer.
		if (((LPUART1 -> CR1) & (0b1 << 14)) != 0) {
			NEOM8N_switch_dma_buffer(1);
		}
		// Clear CM flag.
		LPUART1 -> ICR |= (0b1 << 17);
	}
	// Overrun error interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		LPUART1 -> ICR |= (0b1 << 3);
	}
}

/*** LPUART functions ***/

/* CONFIGURE LPUART1.
 * @param lpuart_use_lse:	Use LSE as clock source if non zero, HSI otherwise.
 * @return:					None.
 */
void LPUART1_init(unsigned char lpuart_use_lse) {
	// Local variables.
	unsigned int lpuart_clock_hz = 0;
	unsigned int brr = 0;
	// Select peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 10); // Reset bits 10-11.
	if (lpuart_use_lse == 0) {
		RCC -> CCIPR |= (0b01 << 10); // LPUART1SEL='01'.
		lpuart_clock_hz = RCC_get_sysclk_khz() * 1000;
	}
	else {
		RCC -> CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
		lpuart_clock_hz = RCC_LSE_FREQUENCY_HZ;
	}
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.
	// Configure power enable pin.
	GPIO_configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	LPUART1_power_off();
	// Configure peripheral.
	LPUART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='0').
	brr = (lpuart_clock_hz * 256);
	brr /= LPUART_BAUD_RATE;
	LPUART1 -> BRR = (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Configure character match interrupt and DMA.
	LPUART1 -> CR2 |= (STRING_CHAR_LF << 24); // LF character used to trigger CM interrupt.
	LPUART1 -> CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
	// Set interrupt priority.
	NVIC_set_priority(NVIC_IT_LPUART1, 1);
	// Enable LPUART1 transmitter and receiver.
	LPUART1 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
}

/* POWER LPUART1 SLAVE ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
LPUART_status_t LPUART1_power_on(void) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
#if (defined HW2_0) || ((defined HW1_0) && (!defined DEBUG))
	// Enable GPIOs.
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Turn NEOM8N on.
	GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
	lptim_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(LPUART_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* POWER LPUART1 SLAVE OFF.
 * @param:			None.
 * @return status:	Function execution status.
 */
void LPUART1_power_off(void) {
	// Turn NEOM8N off.
	GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
#if (defined HW2_0) || ((defined HW1_0) && (!defined DEBUG))
	// Disable LPUART alternate function.
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/* SEND A BYTE THROUGH LOW POWER UART.
 * @param byte_to_send:	Byte to send.
 * @return status:		Function execution status.
 */
LPUART_status_t LPUART1_send_byte(unsigned char tx_byte) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	unsigned int loop_count = 0;
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
