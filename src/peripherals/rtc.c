/*
 * rtc.c
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludovic
 */

#include "rtc.h"

#include "at.h"
#include "exti_reg.h"
#include "gpio_reg.h"
#include "nvic.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "usart_reg.h"

/*** RTC local structures ***/

typedef struct {
	// Calibration flag.
	unsigned char rtc_daily_calibration_done;
	Timestamp rtc_current_timestamp;
	unsigned char rtc_irq_happened;
} RTC_Context;

/*** RTC local global variables ***/

volatile RTC_Context rtc_ctx;

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RTC_IRQHandler(void) {

	/* Alarm A interrupt */
	if (((RTC -> ISR) & (0b1 << 8)) != 0) {
		// Clear flags.
		RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
		// Toggle debug LED.
		GPIOA -> ODR ^= (0b1 << 2);
		rtc_ctx.rtc_irq_happened = 1;
	}

	/* Clear flag of RTC EXTI line (17) */
	EXTI -> PR |= (0b1 << 17);
}

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:	None.
 * @return:	None.
 */
void RTC_EnterInitializationMode(void) {
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	while (((RTC -> ISR) & (0b1 << 6)) == 0); // Wait for INITF='1'.
}

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
void RTC_ExitInitializationMode(void) {
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/* INIT HARDWARE RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void RTC_Init(void) {

	/* Init context */
	rtc_ctx.rtc_daily_calibration_done = 0;
	rtc_ctx.rtc_current_timestamp.year = 0;
	rtc_ctx.rtc_current_timestamp.month = 0;
	rtc_ctx.rtc_current_timestamp.date = 0;
	rtc_ctx.rtc_current_timestamp.hours = 0;
	rtc_ctx.rtc_current_timestamp.minutes = 0;
	rtc_ctx.rtc_current_timestamp.seconds = 0;
	rtc_ctx.rtc_irq_happened = 0;

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
	RTC_EnterInitializationMode();

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
	RTC -> CR |= (0b1 << 5); // Bypass shadow register for read accesses (BYPSHAD='1').
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	RTC -> CR |= (0b1 << 12); // Enable interrupt (ALRAIE='1').
	RTC -> ISR &= ~(0b1 << 8); // Clear flag.

	/* Exit initialization mode */
	RTC_ExitInitializationMode();

	/* Enable RTC alarm interrupt (line 17) */
	EXTI -> IMR |= (0b1 << 17); // IM17='1'.
	EXTI -> RTSR |= (0b1 << 17); // RTC interrupt requires rising edge.

	/* Enable RTC interrupt */
	NVIC_EnableInterrupt(IT_RTC);
}

/* UPDATE RTC CALENDAR WITH A GPS TIMESTAMP.
 * @param gps_timestamp:	Pointer to timestamp from GPS.
 * @return:					None.
 */
void RTC_Calibrate(Timestamp* gps_timestamp) {

	/* Compute register values */
	unsigned int tr_value = 0; // Reset all bits.
	unsigned int dr_value = 0; // Reset all bits.
	// Year.
	unsigned char tens = ((gps_timestamp -> year)-2000) / 10;
	unsigned char units = ((gps_timestamp -> year)-2000) - (tens*10);
	dr_value |= (tens << 20) | (units << 16);
	// Month.
	tens = (gps_timestamp -> month) / 10;
	units = (gps_timestamp -> month) - (tens*10);
	dr_value |= (tens << 12) | (units << 8);
	// Date.
	tens = (gps_timestamp -> date) / 10;
	units = (gps_timestamp -> date) - (tens*10);
	dr_value |= (tens << 4) | (units << 0);
	// Hour.
	tens = (gps_timestamp -> hours) / 10;
	units = (gps_timestamp -> hours) - (tens*10);
	tr_value |= (tens << 20) | (units << 16);
	// Minutes.
	tens = (gps_timestamp -> minutes) / 10;
	units = (gps_timestamp -> minutes) - (tens*10);
	tr_value |= (tens << 12) | (units << 8);
	// Seconds.
	tens = (gps_timestamp -> seconds) / 10;
	units = (gps_timestamp -> seconds) - (tens*10);
	tr_value |= (tens << 4) | (units << 0);

	/* Enter initialization mode */
	RTC_EnterInitializationMode();

	/* Perform update */
	RTC -> TR = tr_value;
	RTC -> DR = dr_value;

	/* Exit initialization mode and restart RTC */
	RTC_ExitInitializationMode();

	/* Update calibration flag */
	rtc_ctx.rtc_daily_calibration_done = 1;
}

/* GET CURRENT RTC CALIBRATION STATUS.
 * @param:	None.
 * @return:	RTC calibration status (0 = not done).
 */
unsigned char RTC_GetCalibrationStatus(void) {
	return rtc_ctx.rtc_daily_calibration_done;
}

/* GET CURRENT RTC TIME.
 * @param rtc_timestamp:	Pointer to timestamp that will contain current RTC time.
 * @return:					None.
 */
void RTC_GetTimestamp(Timestamp* rtc_timestamp) {

	/* Read registers */
	unsigned int dr_value = (RTC -> DR) & 0x00FFFF3F; // Mask reserved bits.
	unsigned int tr_value = (RTC -> TR) & 0x007F7F7F; // Mask reserved bits.

	/* Parse registers into timestamp structure */
	rtc_timestamp -> year = 2000 + ((dr_value & (0b1111 << 20)) >> 20)*10 + ((dr_value & (0b1111 << 16)) >> 16);
	rtc_timestamp -> month = ((dr_value & (0b1 << 12)) >> 12)*10 + ((dr_value & (0b1111 << 8)) >> 8);
	rtc_timestamp -> date = ((dr_value & (0b11 << 4)) >> 4)*10 + (dr_value & 0b1111);
	rtc_timestamp -> hours = ((tr_value & (0b11 << 20)) >> 20)*10 + ((tr_value & (0b1111 << 16)) >> 16);
	rtc_timestamp -> minutes = ((tr_value & (0b111 << 12)) >> 12)*10 + ((tr_value & (0b1111 << 8)) >> 8);
	rtc_timestamp -> seconds = ((tr_value & (0b111 << 4)) >> 4)*10 + (tr_value & 0b1111);
}

unsigned char RTC_GetIrqStatus(void) {
	return rtc_ctx.rtc_irq_happened;
}

void RTC_ClearIrqStatus(void) {
	rtc_ctx.rtc_irq_happened = 0;
}
