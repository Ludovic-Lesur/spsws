/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM functions ***/

void LPTIM1_Init(void);
void LPTIM1_Enable(void);
void LPTIM1_Disable(void);
void LPTIM1_DelayMilliseconds(unsigned int delay_ms, unsigned char stop_mode);

#endif /* LPTIM_H */
