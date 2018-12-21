/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#include "lptim.h"

#include "lptim_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPTIM functions ***/

/* CONFIGURE LPTIM TO COUNT MICROSECONDS.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 31); // LPTIMEN='1'.

	/* Enable peripheral */
	LPTIM1 -> CR |= (0b1 << 0); // ENABLE='1'.

	/* Configure peripheral */
	LPTIM1 -> CFGR = 0; // Reset all bits.
	LPTIM1 -> CFGR |= (0b100 << 9); // Prescaler = 16 (PRESC='100').
	LPTIM1 -> ARR = 0xFFFF; // Maximum period by default.
	LPTIM1 -> CMP = 0; // No compare.
	LPTIM1 -> CNT = 0; // Reset counter.
}

/* START LPTIM.
 * @param period_us:	Timer overflow period in microseconds.
 * @return:				None.
 */
void LPTIM1_Start(unsigned short period_us) {

	/* Enable timer */
	LPTIM1 -> CR |= (0b1 << 0); // EN='1'.

	/* Program period */
	LPTIM1 -> ARR = period_us & 0x0000FFFF;

	/* Start continuous mode */
	LPTIM1 -> CR |= (0b1 << 2); // CNTSTRT='1'.
}

/* STOP LPTIM.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Stop(void) {

	/* Disable timer and stop single mode */
	LPTIM1 -> CR &= ~(0b1 << 2); // CNTSTRT='1'.
	LPTIM1 -> CR &= ~(0b1 << 0); // EN='0'.

	/* Reset counter */
	LPTIM1 -> CNT = 0;
}

/* RETURN LPTIM1 OVERFLOW STATUS.
 * @param:	None.
 * @return:	'1' if timer overflowed, '0' otherwise.
 */
unsigned char LPTIM1_GetArrmFlag(void) {
	// Get ARRM flag value.
	unsigned char arrm_value = 0;
	if (((LPTIM1 -> ISR) & (0b1 << 1)) != 0) {
		arrm_value = 1;
	}
	return arrm_value;
}

/* CLEAR OVERFLOW FLAG.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_ClearArrmFlag(void) {
	// Clear ARRM flag.
	LPTIM1 -> ICR |= (0b1 << 1); // ARRMCF='1'.
}

/* GET NUMBER OF MICROSECONDS ELLAPSED SINCE LPTIM WAS STARTED.
 * @param:	None.
 * @return:	Number of microseconds ellapsed since LPTIM was started.
 */
unsigned int LPTIM1_GetMicroseconds(void) {
	return (LPTIM1 -> CNT);
}
