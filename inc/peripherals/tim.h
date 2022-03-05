/*
 * tim.h
 *
 *  Created on: 4 may 2018
 *      Author: Ludo
 */

#ifndef TIM_H
#define TIM_H

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
	TIM_ERROR_LAST
} TIM_status_t;

/*** TIM functions ***/

void TIM21_init(void);
TIM_status_t TIM21_get_lsi_frequency(unsigned int* lsi_frequency_hz);
void TIM21_disable(void);

void TIM2_init(unsigned short timings[TIM2_TIMINGS_ARRAY_LENGTH]);
void TIM2_enable(void);
void TIM2_disable(void);
void TIM2_start(void);
void TIM2_stop(void);
volatile unsigned int TIM2_get_counter(void);

#define TIM21_status_check(error_base) { if (tim21_status != TIM_SUCCESS) { status = error_base + tim21_status; goto errors; }}

#endif /* TIM_H */
