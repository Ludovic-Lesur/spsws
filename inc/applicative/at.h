/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "neom8n.h"

/*** AT user functions ***/

void AT_Init(void);
void AT_Task(void);

/*** AT utility functions ***/

void AT_FillRxBuffer(unsigned char rx_byte);

// DEBUG
void AT_PrintTimestamp(Timestamp* timestamp_to_print);

#endif /* AT_H */
