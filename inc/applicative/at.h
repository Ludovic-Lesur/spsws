/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "mode.h"
#include "neom8n.h"

#ifdef ATM

/*** AT macros ***/

// Enabled commands.
#define AT_COMMANDS_GPS
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
#define AT_COMMANDS_CW_RSSI
#define AT_COMMANDS_TEST_MODES

/*** AT user functions ***/

void AT_Init(void);
void AT_Task(void);

/*** AT utility functions ***/

void AT_FillRxBuffer(unsigned char rx_byte);

// DEBUG
void AT_PrintTimestamp(Timestamp* timestamp_to_print);

#endif

#endif /* AT_H */
