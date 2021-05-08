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
		NEOM8N_SwitchDmaBuffer(1);
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
void LPUART1_Init(unsigned char lpuart_use_lse) {
	// Select peripheral clock.
	unsigned int lpuart_clock_hz = 0;
	RCC -> CCIPR &= ~(0b11 << 10); // Reset bits 10-11.
	if (lpuart_use_lse == 0) {
		RCC -> CCIPR |= (0b01 << 10); // LPUART1SEL='01'.
		lpuart_clock_hz = RCC_GetSysclkKhz() * 1000;
	}
	else {
		RCC -> CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
		lpuart_clock_hz = RCC_LSE_FREQUENCY_HZ;
	}
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.
	// Configure power enable pin.
	GPIO_Configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_GPS_POWER_ENABLE, 0);
#if (defined HW2_0) || ((defined HW1_0) && (!defined DEBUG))
	// Configure TX and RX GPIOs (first as high impedance).
	GPIO_Configure(&GPIO_LPUART1_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LPUART1_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Configure peripheral.
	LPUART1 -> CR1 &= 0xEC008000; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	LPUART1 -> CR2 &= 0x00F04FEF; // 1 stop bit (STOP='00').
	LPUART1 -> CR3 &= 0xFF0F0836;
	LPUART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='0').
	unsigned int brr = (lpuart_clock_hz * 256);
	brr /= LPUART_BAUD_RATE;
	LPUART1 -> BRR = (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Configure character match interrupt and DMA.
	LPUART1 -> CR2 |= (NMEA_LF << 24); // LF character used to trigger CM interrupt.
	LPUART1 -> CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
	// Set interrupt priority.
	NVIC_SetPriority(NVIC_IT_LPUART1, 1);
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
}

/* UPDATE LPUART BAUD RATE ACCORDING TO CLOCK FREQUENCY.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_UpdateBrr(void) {
	// Local variables.
	unsigned int lpuart_clock_hz = 0;
	// Check clock source.
	if (((RCC -> CCIPR) & (0b11 << 10)) == (0b01 << 10)) {
		// Disable peripheral.
		LPUART1 -> CR1 &= ~(0b1 << 0); // UE='0'.
		// Get current system clock.
		lpuart_clock_hz = RCC_GetSysclkKhz() * 1000;
		// Compute BRR value.
		unsigned int brr = (lpuart_clock_hz * 256);
		brr /= LPUART_BAUD_RATE;
		LPUART1 -> BRR = (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
		// Enable peripheral.
		LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
	}
}

/* ENABLE LPUART TX OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_EnableTx(void) {
	// Enable LPUART1 transmitter.
	LPUART1 -> CR1 |= (0b1 << 3); // Enable transmitter (TE='1').
}

/* ENABLE LPUART RX OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_EnableRx(void) {
	// Enable LPUART1 receiver.
	LPUART1 -> CR1 |= (0b1 << 2); // Enable receiver (RE='1').
	NVIC_EnableInterrupt(NVIC_IT_LPUART1);
}

/* DISABLE LPUART PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_Disable(void) {
	// Disable power control pin.
	GPIO_Configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable LPUART1 peripheral.
	NVIC_DisableInterrupt(NVIC_IT_LPUART1);
	LPUART1 -> CR1 &= ~(0b1 << 2); // Disable transmitter and receiver (TE='0' adnd RE='0').
	LPUART1 -> CR1 &= ~(0b1 << 0);
	// Clear all flags.
	LPUART1 -> ICR |= 0x0012025F;
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 18); // LPUARTEN='0'.
}

/* POWER LPUART1 SLAVE ON.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_PowerOn(void) {
#if (defined HW2_0) || ((defined HW1_0) && (!defined DEBUG))
	// Enable GPIOs.
	GPIO_Configure(&GPIO_LPUART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LPUART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Turn NEOM8N on.
	GPIO_Write(&GPIO_GPS_POWER_ENABLE, 1);
	LPTIM1_DelayMilliseconds(100, 1);
}

/* POWER LPUART1 SLAVE OFF.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_PowerOff(void) {
	// Turn NEOM8N off.
	GPIO_Write(&GPIO_GPS_POWER_ENABLE, 0);
#if (defined HW2_0) || ((defined HW1_0) && (!defined DEBUG))
	// Disable LPUART alternate function.
	GPIO_Configure(&GPIO_LPUART1_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LPUART1_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Delay required if another cycle is requested by applicative layer.
	LPTIM1_DelayMilliseconds(100, 1);
}

/* SEND A BYTE THROUGH LOW POWER UART.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void LPUART1_SendByte(unsigned char tx_byte) {
	// Fill transmit register.
	LPUART1 -> TDR = tx_byte;
	// Wait for transmission to complete.
	unsigned int loop_count = 0;
	while (((LPUART1 -> ISR) & (0b1 << 7)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) break;
	}
}
