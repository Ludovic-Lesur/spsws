/*
 * hwt.c
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludovic
 */

#include "hwt.h"

#include "gpio_reg.h"
#include "pwr_reg.h"
#include "rcc_reg.h"

/*** Hardware Timer structures ***/

typedef struct {
	unsigned char hwt_was_reset_reason; // Used to enable calibration from GPS.
} HWT_Context;

/*** Hardware Timer global variables ***/

static HWT_Context hwt_ctx;

/*** Hardware Timer local functions ***/

/*** Hardware Timer functions ***/

/* INIT HARDWARE TIMER CONTEXT.
 * @param:	None.
 * @return:	None.
 */
void HWT_Init(unsigned char was_reset_reason) {

	/* Init context */
	hwt_ctx.hwt_was_reset_reason = was_reset_reason;
}

/* CHECK IF HARDWARE TIMER TRIGGED THE LAST START-UP.
 * @param:						None.
 * @return was_wakeup_source:	1 if hardware timer was reset reason, 0 otherwise.
 */
unsigned char HWT_WasResetReason(void) {
	return hwt_ctx.hwt_was_reset_reason;
}

/* CHECK IF HARDWARE TIMER EXPIRED.
 * @param:	None.
 * @return:	'1' if hardware timer expired, '0' otherwise.
 */
unsigned char HWT_Expired(void) {
	// Enable GPIOA clock.
	RCC -> IOPENR |= (0b1 << 0);
	// Check HWT output on PA0.
	return (((GPIOA -> IDR) & (0b1 << 0)) >> 0);
}

/* ADJUST DIGITAL POTENTIOMETER TO CALIBRATE HARDWARE TIMER (1 HOUR TARGET)
 * @param hwt_effective_duration:	Effective hardware timer duration in seconds (computed thanks to GPS timestamp).
 * @return:							None.
 */
void HWT_Calibrate(unsigned int hwt_effective_duration_seconds) {

}
