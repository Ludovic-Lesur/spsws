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
	SKY13317_CHANNEL_RF3,
	SKY13317_CHANNEL_LAST
} SKY13317_channel_t;

/*** SKY13317 functions ***/

void SKY13317_init(void);
void SKY13317_disable(void);
void SKY13317_set_channel(SKY13317_channel_t channel);

#endif /* SKY13317_H */
