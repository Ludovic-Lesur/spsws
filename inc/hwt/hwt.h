/*
 * hwt.h
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludovic
 */

#ifndef HWT_HWT_H_
#define HWT_HWT_H_

/*** Hardware Timer functions ***/

void HWT_Init(unsigned char was_reset_reason);
unsigned char HWT_WasResetReason(void);
unsigned char HWT_Expired(void);
void HWT_Calibrate(unsigned int hwt_effective_duration_seconds);

#endif /* HWT_HWT_H_ */
