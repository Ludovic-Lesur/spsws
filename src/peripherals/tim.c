/*
 * tim.c
 *
 *  Created on: 4 may 2018
 *      Author: Ludovic
 */

#include "tim.h"

#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "ultimeter.h"
#include "usart.h"

/*** TIM local global variables ***/

volatile unsigned char ultimeter_seconds_count = 0;

/*** TIM local functions ***/

/* TIM21 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_IRQHandler(void) {

	/* Clear flag */
	TIM21 -> SR &= ~(0b1 << 0); // UIF='0'.

	/* Update counter */
	ultimeter_seconds_count++;

	/* Store wind measurements if period reached */
	if (ultimeter_seconds_count == ULTIMETER_MEASUREMENT_PERIOD_SECONDS) {
		// Store data.
		ULTIMETER_StoreMeasurements();
		// Print data.
		unsigned char mean_speed = 0;
		unsigned char peak_speed = 0;
		unsigned char mean_direction = 0;
		ULTIMETER_GetAverageWindSpeed(&mean_speed);
		ULTIMETER_GetPeakWindSpeed(&peak_speed);
		ULTIMETER_GetAverageWindDirection(&mean_direction);
		USART_SendString("mean_speed=");
		USART_SendValue(mean_speed, USART_Decimal);
		USART_SendString("km/h peak_speed=");
		USART_SendValue(peak_speed, USART_Decimal);
		USART_SendString("km/h mean_direction=");
		USART_SendValue(mean_direction, USART_Decimal);
		USART_SendString("\n");
		// Reset counter.
		ultimeter_seconds_count = 0;
	}
}

/*** TIM functions ***/

/* CONFIGURE TIM21 TO OVERFLOW EVERY SECOND.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.

	/* Reset timer before configuration */
	TIM21 -> CR1 &= ~(0b1 << 0); // Disable TIM21 (CEN='0').
	TIM21 -> CNT = 0; // Reset counter.

	/* Configure TIM21 as master to count milliseconds and overflow every seconds */
	TIM21 -> PSC = SYSCLK_KHZ; // Timer is clocked by SYSCLK (see RCC_Init() function). SYSCLK_KHZ-1 ?
	TIM21 -> ARR = 1000; // 999 ?
	TIM21 -> CR2 &= ~(0b111 << 4); // Reset bits 4-6.
	TIM21 -> CR2 |= (0b010 << 4); // Generate trigger on update event (MMS='010').

#if (defined CM_RTC || defined ATM)
	/* Enable interrupt for wind speed/direction measurements */
	TIM21 -> DIER |= (0b1 << 0); // UIE='1'.
#endif

	/* Generate event to update registers */
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.

	/* Start timer */
	TIM21 -> CR1 |= (0b1 << 0); // Enable TIM21 (CEN='1').
}

/* CONFIGURE TIM22 TO COUNT SECONDS SINCE START-UP.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 5); // TIM22EN='1'.

	/* Reset timer before configuration */
	TIM22 -> CR1 &= ~(0b1 << 0); // Disable TIM22 (CEN='0').
	TIM22 -> CNT = 0; // Reset counter.

	/* Configure TIM22 as slave to count seconds */
	TIM22 -> SMCR &= ~(0b111 << 4); // TS = '000' to select ITR0 = TIM1 as trigger input.
	TIM22 -> SMCR |= (0b111 << 0); // SMS = '111' to enable slave mode with external clock.

	/* Generate event to update registers */
	TIM22 -> EGR |= (0b1 << 0); // UG='1'.

	/* Start timers */
	TIM22 -> CR1 |= (0b1 << 0); // Enable TIM22 (CEN='1').
}

/* RETURNS THE NUMBER OF SECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of seconds ellapsed since start-up.
 */
unsigned int TIM22_GetSeconds(void) {
	return (TIM22 -> CNT);
}

/* RETURNS THE NUMBER OF MILLISECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of milliseconds ellapsed since start-up.
 */
unsigned int TIM22_GetMilliseconds(void) {
	return ((TIM22 -> CNT)*1000 + (TIM21 -> CNT));
}

/* DELAY FUNCTION.
 * @param msToWait:	Number of milliseconds to wait.
 * @return:			None.
 */
void TIM22_WaitMilliseconds(unsigned int ms_to_wait) {
	unsigned int start_ms = TIM22_GetMilliseconds();
	while (TIM22_GetMilliseconds() < (start_ms + ms_to_wait));
}

/* CONFIGURE TIM2 FOR ULTIMETER PHASE SHIFT MEASURE OR SIGFOX BPSK MODULATION.
 * @param mode:		Timer mode (see Timer2_Mode enumeration in tim.h).
 * @param timings:	Events timings given as [ARR, CCR1, CCR2, CCR3, CCR4].
 * @return:			None.
 */
void TIM2_Init(TIM2_Mode mode, unsigned short timings[TIM2_TIMINGS_ARRAY_LENGTH]) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.

	/* Reset timer before configuration */
	TIM2 -> CR1 = 0; // Disable TIM2 (CEN='0').
	TIM2 -> CNT = 0; // Reset counter.
	TIM2 -> CR1 |= (0b1 << 2); // UIF set only on counter overflow (URS='1').

	/* Init common variables */
	unsigned int arr_value = 0;
	unsigned int psc_value = 0;

	switch (mode) {

	case TIM2_MODE_ULTIMETER:
		/* Configure TIM2 to overflow every (ULTIMETER_MEASUREMENT_PERIOD_SECONDS+1) seconds */
		arr_value = 0xFFFF; // Maximum overflow value for the desired period (to optimize "dynamic" = accuracy).
		TIM2 -> ARR = arr_value;
		// PSC = (desired_period * timer_input_clock) / (ARR).
		psc_value = ((ULTIMETER_MEASUREMENT_PERIOD_SECONDS + 1) * (SYSCLK_KHZ*1000)) / (arr_value);
		if (psc_value > 0xFFFF) {
			psc_value = 0xFFFF;
		}
		TIM2 -> PSC = psc_value; // Timer is clocked by SYSCLK (see RCC_Init() function).
		break;

	case TIM2_MODE_SIGFOX:
		/* Configure TIM2 to overflow every timing[0] microseconds */
		TIM2 -> PSC = 15; // Timer input clock = SYSCLK / (PSC + 1) = 1MHz.
		TIM2 -> ARR = timings[TIM2_TIMINGS_ARRAY_ARR_IDX];
		// Configure events timestamps.
		TIM2 -> CCR1 = timings[TIM2_TIMINGS_ARRAY_CCR1_IDX];
		TIM2 -> CCR2 = timings[TIM2_TIMINGS_ARRAY_CCR2_IDX];
		TIM2 -> CCR3 = timings[TIM2_TIMINGS_ARRAY_CCR3_IDX];
		TIM2 -> CCR4 = timings[TIM2_TIMINGS_ARRAY_CCR4_IDX];
		// Enable channels (OCxm='001').
		TIM2 -> CCMR1 |= (0b001 << 12) | (0b011 << 4);
		TIM2 -> CCMR2 |= (0b001 << 12) | (0b011 << 4);
		// Generate event to update registers.
		TIM21 -> EGR |= (0b1 << 0); // UG='1'.
		// Enable update and CCRx interrupts.
		TIM2 -> DIER |= (0b11111 << 0);
		NVIC_EnableInterrupt(IT_TIM2);
		break;

	default:
		break;
	}
}

/* START TIM2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Start(void) {
	// Reset counter value and Enable TIM2.
	TIM2 -> CNT = 0;
	TIM2 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* STOP AND RESET TIM2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Stop(void) {
	// Disable TIM2.
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* RETURN CURRENT TIM2 COUNTER VALUE.
 * @param:	None.
 * @return:	Current TIM2 counter value.
 */
volatile unsigned int TIM2_GetCounter(void) {
	return (TIM2 -> CNT);
}
