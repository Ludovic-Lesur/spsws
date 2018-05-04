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
unsigned int TIM_TimeGetS(void);
unsigned int TIM_TimeGetMs(void);
void TIM_TimeWaitMs(unsigned int ms_to_wait);

#endif /* PERIPHERALS_TIM_H */
