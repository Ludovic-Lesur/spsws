/*
 * tim.h
 *
 *  Created on: 4 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_TIM_H
#define PERIPHERALS_TIM_H

/*** TIM functions ***/

void TIM21_Init(void);

void TIM22_Init(void);
unsigned int TIM22_GetSeconds(void);
unsigned int TIM22_GetMilliseconds(void);
void TIM22_WaitMilliseconds(unsigned int ms_to_wait);

void TIM2_Init(void);
void TIM2_Restart(void);
void TIM2_Stop(void);
volatile unsigned int TIM2_GetCounter(void);

#endif /* PERIPHERALS_TIM_H */
