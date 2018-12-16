/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM functions ***/

void LPTIM1_Init(void);
void LPTIM1_Start(unsigned short period_us);
unsigned char LPTIM1_GetArrmFlag(void);
void LPTIM1_ClearArrmFlag(void);
void LPTIM1_Stop(void);
unsigned short LPTIM1_GetMicroseconds(void);

#endif /* LPTIM_H */
