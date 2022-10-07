/*
 * sky13317.h
 *
 *  Created on: 15 dec. 2018
 *      Author: Ludo
 */

#ifndef __SKY13317_H__
#define __SKY13317_H__

/*** SKY13317 structures ***/

typedef enum {
	SKY13317_SUCCESS,
	SKY13317_ERROR_CHANNEL,
	SKY13317_ERROR_BASE_LAST = 0x0100
} SKY13317_status_t;

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
SKY13317_status_t SKY13317_set_channel(SKY13317_channel_t channel);

#endif /* __SKY13317_H__ */
