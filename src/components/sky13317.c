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

	/* Configure GPIOs */
#ifdef HW1_0
	GPIO_Configure(&GPIO_RF_CHANNEL_A, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_CHANNEL_A, 0);
	GPIO_Configure(&GPIO_RF_CHANNEL_B, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
	GPIO_Configure(&GPIO_RF_TX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_TX_ENABLE, 0);
	GPIO_Configure(&GPIO_RF_RX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_RX_ENABLE, 0);
#endif
}

/* DISABLE RF SWITCH GPIOs.
 * @param:	None.
 * @return:	None.
 */
void SKY13317_DisableGpio(void) {
#ifdef HW1_0
	GPIO_Configure(&GPIO_RF_CHANNEL_A, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_CHANNEL_A, 0);
	GPIO_Configure(&GPIO_RF_CHANNEL_B, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
	GPIO_Configure(&GPIO_RF_TX_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_TX_ENABLE, 0);
	GPIO_Configure(&GPIO_RF_RX_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_RX_ENABLE, 0);
#endif
}

/* SELECT RF SWITCH CHANNEL.
 * @param channel:	Channel to select (see SKY13317_Channel enumeration in sky13317.h).
 * ]return:			None.
 */
void SKY13317_SetChannel(SKY13317_Channel channel) {

	/* Reset channels */
#ifdef HW1_0
	GPIO_Write(&GPIO_RF_CHANNEL_A, 0);
	GPIO_Write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
	GPIO_Write(&GPIO_RF_TX_ENABLE, 0);
	GPIO_Write(&GPIO_RF_RX_ENABLE, 0);
#endif

	/* Select channel */
	switch (channel) {

	case SKY13317_CHANNEL_NONE:
		// Allready done by previous reset.
		break;

	case SKY13317_CHANNEL_RF1:
#ifdef HW1_0
		GPIO_Write(&GPIO_RF_CHANNEL_A, 1);
#endif
#ifdef HW2_0
		// Not connected.
#endif
		break;

	case SKY13317_CHANNEL_RF2:
#ifdef HW1_0
		GPIO_Write(&GPIO_RF_CHANNEL_A, 1);
		GPIO_Write(&GPIO_RF_CHANNEL_B, 1);
#endif
#ifdef HW2_0
		GPIO_Write(&GPIO_RF_TX_ENABLE, 1);
#endif
		break;

	case SKY13317_CHANNEL_RF3:
#ifdef HW1_0
		GPIO_Write(&GPIO_RF_CHANNEL_B, 1);
#endif
#ifdef HW2_0
		GPIO_Write(&GPIO_RF_RX_ENABLE, 1);
#endif
		break;

	default:
		break;
	}
}

