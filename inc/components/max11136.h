/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef MAX11136_H
#define MAX11136_H

/*** MAX11136 macros ***/

// Full scale on 12-bits.
#define MAX11136_FULL_SCALE		4095

// Channels definition.
#define MAX11136_CHANNEL_AIN0	0
#define MAX11136_CHANNEL_AIN1	1
#define MAX11136_CHANNEL_AIN2	2
#define MAX11136_CHANNEL_AIN3	3
#define MAX11136_CHANNEL_AIN4	4
#define MAX11136_CHANNEL_AIN5	5
#define MAX11136_CHANNEL_AIN6	6
#define MAX11136_CHANNEL_AIN7	7

/*** MAX11136 functions ***/

void MAX11136_Init(void);
unsigned char MAX11136_PerformMeasurements(void);
void MAX11136_GetChannel(unsigned char channel, unsigned int* channel_result_12bits);

#endif /* MAX11136_H */
