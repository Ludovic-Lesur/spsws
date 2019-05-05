/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM functions ***/

void LPTIM1_Init(unsigned char lptim1_use_lsi);
void LPTIM1_Start(void);
void LPTIM1_Stop(void);
void LPTIM1_Disable(void);
void LPTIM1_DelayMilliseconds(unsigned int delay_ms);

#endif /* LPTIM_H */
