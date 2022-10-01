/*
 * tim.h
 *
 *  Created on: 4 may 2018
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#include "types.h"

/*** TIM macros ***/

#define TIM2_TIMINGS_ARRAY_LENGTH		5
#define TIM2_TIMINGS_ARRAY_ARR_IDX		0
#define TIM2_TIMINGS_ARRAY_CCR1_IDX		1
#define TIM2_TIMINGS_ARRAY_CCR2_IDX		2
#define TIM2_TIMINGS_ARRAY_CCR3_IDX		3
#define TIM2_TIMINGS_ARRAY_CCR4_IDX		4

/*** TIM structures ***/

typedef enum {
	TIM_SUCCESS = 0,
	TIM_ERROR_INTERRUPT_TIMEOUT,
	TIM_ERROR_BASE_LAST = 0x0100
} TIM_status_t;

/*** TIM functions ***/

void TIM21_init(void);
TIM_status_t TIM21_get_lsi_frequency(uint32_t* lsi_frequency_hz);
void TIM21_disable(void);

void TIM2_init(uint16_t timings[TIM2_TIMINGS_ARRAY_LENGTH]);
void TIM2_start(void);
void TIM2_stop(void);
volatile uint32_t TIM2_get_counter(void);

#define TIM21_status_check(error_base) { if (tim21_status != TIM_SUCCESS) { status = error_base + tim21_status; goto errors; }}
#define TIM21_error_check() { ERROR_status_check(tim21_status, TIM_SUCCESS, ERROR_BASE_TIM21); }
#define TIM21_error_check_print() { ERROR_status_check_print(tim21_status, TIM_SUCCESS, ERROR_BASE_TIM21); }

#endif /* __TIM_H__ */
