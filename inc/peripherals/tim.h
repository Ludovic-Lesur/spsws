/*
 * tim.h
 *
 *  Created on: 4 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_TIM_H
#define PERIPHERALS_TIM_H

/*** TIM functions ***/

void TIM_TimeInit(void);
unsigned int TIM_TimeGetSeconds(void);
unsigned int TIM_TimeGetMilliseconds(void);
void TIM_TimeWaitMilliseconds(unsigned int ms_to_wait);

#endif /* PERIPHERALS_TIM_H */
