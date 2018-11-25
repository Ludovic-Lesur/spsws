/*
 * rtc.c
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludovic
 */

#include "rtc.h"

#include "gpio_reg.h"
#include "nvic.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "rtc_reg.h"

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RTC_IRQHandler(void) {

	/* Alarm A interrupt */
	if (((RTC -> ISR) & (0b1 << 8)) != 0) {
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
		// Toggle debug LED.
		GPIOA -> ODR ^= (0b1 << 2);
	}
}

/*** RTC functions ***/

/* INIT HARDWARE RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void RTC_Init(void) {

	/* Enable LSE (32.768kHz crystal) */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	PWR -> CR |= (0b1 << 8); // Set DBP bit to unlock RCC register write protection.
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	while (((RCC -> CSR) & (0b1 << 9)) == 0); // Wait for LSE to be stable (LSERDY='1').

	/* Select RTC clock source */
	RCC -> CSR &= ~(0b11 << 16); // Reset bits 16-17.
	RCC -> CSR |= (0b01 << 16); // Select LSE (RTCSEL='01').

	/* Enable RTC */
	RCC -> CSR |= (0b1 << 18); // RTCEN='1'.

	/* Unlock RTC registers */
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	while (((RTC -> ISR) & (0b1 << 6)) == 0); // Wait for INITF='1'.

	/* Configure prescaler */
	// Prescaler settings to get 1Hz: PREDIV_A=127 and PREDIV_S=255 (128*256 = 32768).
	RTC -> PRER = 0; // Reset all bits.
	RTC -> PRER |= (127 << 16) | (255 << 0);

	/* Reset calendar to Saturday January 1st 2000 00:00:01 */
	RTC -> TR = 0x00000001; // Add 1 second to prevent from alarm interrupt at start-up.
	RTC -> DR = 0x0000C101;

	/* Configure Alarm A to wake-up MCU every hour */
	RTC -> ALRMAR = 0; // Reset all bits.
	RTC -> ALRMAR |= (0b1 << 31) | (0b1 << 23); // Mask day and hour (only check minutes and seconds).
	RTC -> ALRMAR |= (0b1 << 15); // Mask minutes (debug).
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	RTC -> CR |= (0b1 << 12); // Enable interrupt (ALRAIE='1').
	RTC -> ISR &= ~(0b1 << 8); // Clear flag.

	/* Exit initialization mode */
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.

	/* Enable RTC interrupt */
	NVIC_EnableInterrupt(IT_RTC);
}

/* UPDATE RTC CALENDAR WITH A GPS TIMESTAMP.
 * @param gps_timestamp:	Timestamp from GPS.
 * @return:					None.
 */
void RTC_Calibrate(GPS_TimestampData* gps_timestamp) {

}
