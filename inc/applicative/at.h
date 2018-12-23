/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

/*** AT user functions ***/

void AT_Init(void);
void AT_Task(void);

/*** AT utility functions ***/

void AT_FillRxBuffer(unsigned char rx_byte);

#endif /* AT_H */
