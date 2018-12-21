/*
 * tim.h
 *
 *  Created on: 4 may 2018
 *      Author: Ludovic
 */

#ifndef TIM_H
#define TIM_H

/*** TIM structures ***/

typedef enum {
	TIM2_MODE_ULTIMETER,
	TIM2_MODE_SIGFOX
} TIM2_Mode;

/*** TIM macros ***/

#define TIM2_TIMINGS_ARRAY_LENGTH		5
#define TIM2_TIMINGS_ARRAY_ARR_IDX		0
#define TIM2_TIMINGS_ARRAY_CCR1_IDX		1
#define TIM2_TIMINGS_ARRAY_CCR2_IDX		2
#define TIM2_TIMINGS_ARRAY_CCR3_IDX		3
#define TIM2_TIMINGS_ARRAY_CCR4_IDX		4

/*** TIM functions ***/

void TIM21_Init(void);

void TIM22_Init(void);
unsigned int TIM22_GetSeconds(void);
unsigned int TIM22_GetMilliseconds(void);
void TIM22_WaitMilliseconds(unsigned int ms_to_wait);

void TIM2_Init(TIM2_Mode mode, unsigned short timings[TIM2_TIMINGS_ARRAY_LENGTH]);
void TIM2_Start(void);
void TIM2_Stop(void);
volatile unsigned int TIM2_GetCounter(void);

#endif /* TIM_H */
