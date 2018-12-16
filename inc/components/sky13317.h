/*
 * sky13317.h
 *
 *  Created on: 15 dec. 2018
 *      Author: Ludo
 */

#ifndef SKY13317_H
#define SKY13317_H

/*** SKY13317 structures ***/

typedef enum {
	SKY13317_CHANNEL_NONE,
	SKY13317_CHANNEL_RF1,
	SKY13317_CHANNEL_RF2,
	SKY13317_CHANNEL_RF3
} SKY13317_Channel;

/*** SKY13317 functions ***/

void SKY13317_Init(void);
void SKY13317_SetChannel(SKY13317_Channel channel);

#endif /* SKY13317_H */
