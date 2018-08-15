/*
 * hwt.c
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludovic
 */

#include "hwt.h"

#include "gpio_reg.h"
#include "mcp4162.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "tim.h"

/*** Hardware timer local macros ***/

#define HWT_TARGETTED_DURATION_SECONDS		3600	// HWT targetted duration in seconds.
#define HWT_ZERO_FEEDBACK_ERROR_SECONDS		60 		// No feedback applied if duration error is under this value.
#define HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS	1800	// Maximum feedback is applied as soon as duration error exceeds this value.
#define HWT_MAXIMUM_FEEDBACK_STEPS			5		// Maximum feedback value expressed in digital potentiometer steps.

/*** Hardware Timer structures ***/

typedef struct {
	unsigned char hwt_was_reset_reason; // Used to enable calibration from GPS.
	unsigned int hwt_absolute_error_seconds;
	unsigned char hwt_feedback_direction; // 0 = duration error is negative (> 3600s) , 1 = duration error is positive (< 3600s).
	unsigned char hwt_feedback_value_steps;
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
	hwt_ctx.hwt_absolute_error_seconds = 0;
	hwt_ctx.hwt_feedback_direction = 0;
	hwt_ctx.hwt_feedback_value_steps = 0;

	/* Reset HWT */
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 6); // Reset bits 6-7.
	GPIOB -> MODER |= (0b01 << 6);
	// Close relay, wait 2s while capacitor is charged and release relay.
	GPIOB -> ODR |= (0b1 << 3);
	TIM_TimeWaitMilliseconds(2000);
	GPIOB -> ODR &= ~(0b1 << 3);

	/* Init digital potentiometer */
	MCP4162_Init();
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
	// Compute error.
	signed int hwt_duration_error_seconds = HWT_TARGETTED_DURATION_SECONDS-hwt_effective_duration_seconds;
	// Compute feedback direction.
	if (hwt_duration_error_seconds < 0) {
		hwt_ctx.hwt_feedback_direction = 0;
		hwt_ctx.hwt_absolute_error_seconds = (-1)*hwt_duration_error_seconds;
	}
	else {
		hwt_ctx.hwt_feedback_direction = 1;
		hwt_ctx.hwt_absolute_error_seconds = hwt_duration_error_seconds;
	}
	// Implement numeric proportional control.
	if (hwt_ctx.hwt_absolute_error_seconds > HWT_ZERO_FEEDBACK_ERROR_SECONDS) {
		// Feedback is required, compute number of steps.
		if (hwt_ctx.hwt_absolute_error_seconds > HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS) {
			// Feedback saturation.
			hwt_ctx.hwt_feedback_value_steps = HWT_MAXIMUM_FEEDBACK_STEPS;
		}
		else {
			// Linear feedback.
			hwt_ctx.hwt_feedback_value_steps = 1 + (((HWT_MAXIMUM_FEEDBACK_STEPS-1)*(hwt_ctx.hwt_absolute_error_seconds-HWT_ZERO_FEEDBACK_ERROR_SECONDS)) / (HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS));
		}
		// Apply feedback.
		unsigned char step_idx = 0;
		for (step_idx=0 ; step_idx<hwt_ctx.hwt_feedback_value_steps ; step_idx++) {
			if (hwt_ctx.hwt_feedback_direction == 0) {
				// Voltage reference is too low -> increase potentiometer value.
				MCP4162_Increment();
			}
			else {
				// Voltage reference is too high -> decrease potentiometer value.
				MCP4162_Decrement();
			}
		}
	}
}

#ifdef HARDWARE_TIMER
/* GET HWT PARAMETERS.
 * @param hwt_absolute_error:		Pointer to int that will contain HWT absolute error in seconds.
 * @param hwt_feedback_value:		Pointer to char that will contain HWT feedback value expressed as a number of steps.
 * @param hwt_feedback_direction:	Pointer to char that will contain HWT feedback direction (0/1).
 * @return:							None.
 */
void HWT_GetParameters(unsigned int* hwt_absolute_error, unsigned char* hwt_feedback_value, unsigned char* hwt_feedback_direction) {
	(*hwt_absolute_error) = hwt_ctx.hwt_absolute_error_seconds;
	(*hwt_feedback_value) = hwt_ctx.hwt_feedback_value_steps;
	(*hwt_feedback_direction) = hwt_ctx.hwt_feedback_direction;
}
#endif
