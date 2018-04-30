/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_LPTIM_H
#define PERIPHERALS_LPTIM_H

/*** LPTIM functions ***/

void LPTIM_Init(void);
void LPTIM_WaitMs(unsigned int ms_to_wait);

#endif /* PERIPHERALS_LPTIM_H */
