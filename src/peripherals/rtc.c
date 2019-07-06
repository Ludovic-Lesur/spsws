/*
 * rtc.c
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#include "rtc.h"

#include "at.h"
#include "exti_reg.h"
#include "mode.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "tim.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_SECONDS	5

/*** RTC local global variables ***/

#if (defined CM || defined ATM)
volatile unsigned char rtc_alra_flag;
volatile unsigned char rtc_alrb_flag;
#endif

/*** RTC local functions ***/

#if (defined CM || defined ATM)
/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RTC_IRQHandler(void) {

	/* Alarm A interrupt */
	if (((RTC -> ISR) & (0b1 << 8)) != 0) {
		// Update flag
		rtc_alra_flag = 1;
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
	}

	/* Alarm B interrupt */
	if (((RTC -> ISR) & (0b1 << 9)) != 0) {
		// Update flag
		rtc_alrb_flag = 1;
		// Clear flag.
		RTC -> ISR &= ~(0b1 << 9); // ALRNF='0'.
	}

	/* Clear EXTI flag */
	EXTI -> PR |= (0b1 << 17);
}
#endif

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:						None.
 * @return rtc_initf_success:	1 if RTC entered initialization mode, 0 otherwise.
 */
unsigned char RTC_EnterInitializationMode(void) {
	// Local variables.
	unsigned char rtc_initf_success = 1;
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	unsigned int loop_start_time = TIM22_GetSeconds();
	while (((RTC -> ISR) & (0b1 << 6)) == 0) {
		// Wait for INITF='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + RTC_INIT_TIMEOUT_SECONDS)) {
			rtc_initf_success = 0;
			break;
		}
	}
	return rtc_initf_success;
}

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
void RTC_ExitInitializationMode(void) {
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/* RESET RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void RTC_Reset(void) {

	/* Reset RTC peripheral */
	RCC -> CSR |= (0b1 << 19); // RTCRST='1'.
	unsigned char j = 0;
	for (j=0 ; j<100 ; j++);
	RCC -> CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/* INIT HARDWARE RTC PERIPHERAL.
 * @param rtc_use_lse:	RTC will be clocked on LSI if 0, on LSE otherwise.
 * @PARAM lsi_freq_hz:	Effective LSI oscillator frequency used to compute the accurate prescaler value (only if LSI is used as source).
 * @return:				None.
 */
void RTC_Init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz) {

	/* Manage RTC clock source */
	if ((*rtc_use_lse) != 0) {
		// Use LSE.
		RCC -> CSR |= (0b01 << 16); // RTCSEL='01'.
	}
	else {
		// Use LSI.
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
	}

	/* Enable RTC and register access */
	RCC -> CSR |= (0b1 << 18); // RTCEN='1'.

	/* Switch to LSI if RTC failed to enter initialization mode */
	if (RTC_EnterInitializationMode() == 0) {
		RTC_Reset();
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
		RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
		RTC_EnterInitializationMode();
		// Update flag.
		(*rtc_use_lse) = 0;
	}

	/* Configure prescaler */
	if ((*rtc_use_lse) != 0) {
		// LSE frequency is 32.768kHz typical.
		RTC -> PRER = (127 << 16) | (255 << 0); // PREDIV_A=127 and PREDIV_S=255 (128*256 = 32768).
	}
	else {
		// Compute prescaler according to measured LSI frequency.
		RTC -> PRER = (127 << 16) | (((lsi_freq_hz / 128) - 1) << 0); // PREDIV_A=127 and PREDIV_S=((lsi_freq_hz/128)-1).
	}

	/* Bypass shadow registers */
	RTC -> CR |= (0b1 << 5); // BYPSHAD='1'.

	/* Configure alarm A to wake-up MCU every hour */
	RTC -> ALRMAR = 0; // Reset all bits.
	RTC -> ALRMAR |= (0b1 << 31) | (0b1 << 23); // Mask day and hour (only check minutes and seconds).
	//RTC -> ALRMAR |= (0b1 << 15); // Mask minutes (to wake-up every minute for debug).
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	RTC -> CR |= (0b1 << 12); // Enable interrupt (ALRAIE='1').
	RTC -> ISR &= ~(0b1 << 8); // Clear flag.

#ifdef IM
	/* Disable alarm B */
	RTC -> ALRMBR = 0;
	RTC -> CR &= ~(0b1 << 9); // Disable Alarm B.
	RTC -> CR &= ~(0b1 << 13); // Disable interrupt (ALRBIE='0').
	RTC -> ISR &= ~(0b1 << 9); // Clear flag.
#endif
#if (defined CM || defined ATM)
	/* Configure alarm B to wake-up every seconds (wind measurements) */
	RTC -> ALRMBR = 0;
	RTC -> ALRMBR |= (0b1 << 31) | (0b1 << 23) | (0b1 << 15) | (0b1 << 7); // Mask all fields.
	RTC -> CR |= (0b1 << 9); // Enable Alarm B.
	RTC -> CR |= (0b1 << 13); // Enable interrupt (ALRBIE='1').
	RTC -> ISR &= ~(0b1 << 9); // Clear flag.
#endif

	/* Exit initialization mode */
	RTC_ExitInitializationMode();

	/* Enable RTC alarm interrupt (line 17) */
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	EXTI -> IMR |= (0b1 << 17); // IM17='1'.
	EXTI -> RTSR |= (0b1 << 17); // RTC interrupt requires rising edge.
	EXTI -> PR |= (0b1 << 17); // Clear flag.
}

#if (defined CM || defined ATM)
/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile unsigned char RTC_GetAlarmAFlag(void) {
	return rtc_alra_flag;
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile unsigned char RTC_GetAlarmBFlag(void) {
	return rtc_alrb_flag;
}
#endif

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_ClearAlarmAFlag(void) {
	// Clear ALARM and EXTI flags.
	RTC -> ISR &= ~(0b1 << 8); // Clear flags.
	EXTI -> PR |= (0b1 << 17);
#if (defined CM || defined ATM)
	rtc_alra_flag = 0;
#endif
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_ClearAlarmBFlag(void) {
	// Clear ALARM and EXTI flags.
	RTC -> ISR &= ~(0b1 << 9); // Clear flags.
	EXTI -> PR |= (0b1 << 17);
#if (defined CM || defined ATM)
	rtc_alrb_flag = 0;
#endif
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
	rtc_timestamp -> year = 2000 + ((dr_value & (0b1111 << 20)) >> 20) * 10 + ((dr_value & (0b1111 << 16)) >> 16);
	rtc_timestamp -> month = ((dr_value & (0b1 << 12)) >> 12) * 10 + ((dr_value & (0b1111 << 8)) >> 8);
	rtc_timestamp -> date = ((dr_value & (0b11 << 4)) >> 4) * 10 + (dr_value & 0b1111);
	rtc_timestamp -> hours = ((tr_value & (0b11 << 20)) >> 20) * 10 + ((tr_value & (0b1111 << 16)) >> 16);
	rtc_timestamp -> minutes = ((tr_value & (0b111 << 12)) >> 12) * 10 + ((tr_value & (0b1111 << 8)) >> 8);
	rtc_timestamp -> seconds = ((tr_value & (0b111 << 4)) >> 4) * 10 + (tr_value & 0b1111);
}
