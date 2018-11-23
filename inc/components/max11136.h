/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludovic
 */

#ifndef MAX11136_H
#define MAX11136_H

/*** MAX11136 macros ***/

#define MAX11136_NUMBER_OF_CHANNELS	8
#define MAX11136_RESOLUTION			12

/*** MAX11136 functions ***/

void MAX11136_Init(void);
void MAX11136_ConvertAllChannels(unsigned short* max11136_result);

#endif /* MAX11136_H */
