/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludovic
 */

#ifndef MAX11136_H
#define MAX11136_H

/*** MAX11136 macros ***/

#define MAX11136_FULL_SCALE				4095 // 12-bits result.
#define MAX11136_BANDGAP_VOLTAGE_MV		2048 // Bandgap reference attached to AIN7.

/*** MAX11136 functions ***/

void MAX11136_Init(void);
unsigned char MAX11136_ConvertAllChannels(void);
void MAX11136_GetChannelVoltage12bits(unsigned char channel, unsigned short* channel_voltage_12bits);

#endif /* MAX11136_H */
