/*
 * sky13317.c
 *
 *  Created on: 15 dec. 2018
 *      Author: Ludo
 */

#include "sky13317.h"

#include "gpio.h"
#include "mapping.h"

/*** SKY13317 functions ***/

/* INIT SKY13317 RF SWITCH.
 * @param:	None.
 * @return:	None.
 */
void SKY13317_Init(void) {

	/* Configure RF channels A/B */
	GPIO_Configure(GPIO_RF_CHANNEL_A, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Write(GPIO_RF_CHANNEL_A, 0);
	GPIO_Configure(GPIO_RF_CHANNEL_B, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Write(GPIO_RF_CHANNEL_B, 0);
}

/* SELECT RF SWITCH CHANNEL.
 * @param channel:	Channel to select (see SKY13317_Channel enumeration in sky13317.h).
 * ]return:			None.
 */
void SKY13317_SetChannel(SKY13317_Channel channel) {

	/* Reset channels */
	GPIO_Write(GPIO_RF_CHANNEL_A, 0);
	GPIO_Write(GPIO_RF_CHANNEL_B, 0);

	/* Select channel */
	switch (channel) {

	case SKY13317_CHANNEL_NONE:
		// Allready done by previous reset.
		break;

	case SKY13317_CHANNEL_RF1:
		// PA3='1' and PA4='0'.
		GPIO_Write(GPIO_RF_CHANNEL_A, 1);
		break;

	case SKY13317_CHANNEL_RF2:
		// PA3='1' and PA4='1'.
		GPIO_Write(GPIO_RF_CHANNEL_A, 1);
		GPIO_Write(GPIO_RF_CHANNEL_B, 1);
		break;

	case SKY13317_CHANNEL_RF3:
		// PA3='0' and PA4='1'.
		GPIO_Write(GPIO_RF_CHANNEL_B, 1);
		break;

	default:
		break;
	}
}

