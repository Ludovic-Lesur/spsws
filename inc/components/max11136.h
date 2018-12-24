/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef MAX11136_H
#define MAX11136_H

/*** MAX11136 macros ***/

// Channels mapping.
#ifdef HW1_0
#define MAX11136_CHANNEL_AIN0			0
#define MAX11136_CHANNEL_AIN1			1
#define MAX11136_CHANNEL_AIN2			2
#define MAX11136_CHANNEL_AIN3			3
#define MAX11136_CHANNEL_SOLAR_PANEL	4
#define MAX11136_CHANNEL_SUPERCAP		5
#define MAX11136_CHANNEL_LDR			6
#define MAX11136_CHANNEL_BANDGAP		7
#endif

/*** MAX11136 functions ***/

void MAX11136_Init(void);
unsigned char MAX11136_PerformMeasurements(void);
void MAX11136_GetSupplyVoltage(unsigned int* supply_voltage_mv);
void MAX11136_GetChannelVoltage(unsigned char channel, unsigned int* channel_voltage_mv);

#endif /* MAX11136_H */
