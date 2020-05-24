/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM structures ***/

typedef enum {
	LPTIM_MODE_LSI_CALIBRATION,
	LPTIM_MODE_DELAY,
	LPTIM_MODE_ULTIMETER
} LPTIM_Mode;

/*** LPTIM functions ***/

void LPTIM1_Init(LPTIM_Mode lptim1_mode);
void LPTIM1_Start(void);
void LPTIM1_Stop(void);
void LPTIM1_Enable(void);
void LPTIM1_Disable(void);
void LPTIM1_DelayMilliseconds(unsigned int delay_ms);
volatile unsigned int LPTIM1_GetCounter(void);

#endif /* LPTIM_H */
