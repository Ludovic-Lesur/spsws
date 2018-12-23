/*
 * hwt.h
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludo
 */

#ifndef HWT_H
#define HWT_H

#include "neom8n.h"

/*** Hardware Timer functions ***/

unsigned char HWT_Expired(void);
void HWT_Reset(void);
void HWT_Process(unsigned char was_wake_up_reason, unsigned char timestamp_retrieved, Timestamp* gps_timestamp);

#endif /* HWT_H */
