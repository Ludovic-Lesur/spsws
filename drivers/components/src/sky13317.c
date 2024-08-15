/*
 * sky13317.c
 *
 *  Created on: 15 dec. 2018
 *      Author: Ludo
 */

#include "sky13317.h"

#include "gpio.h"
#include "gpio_mapping.h"

/*** SKY13317 functions ***/

/*******************************************************************/
void SKY13317_init(void) {
	// Configure GPIOs.
#ifdef HW1_0
	GPIO_configure(&GPIO_RF_CHANNEL_A, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_RF_CHANNEL_A, 0);
	GPIO_configure(&GPIO_RF_CHANNEL_B, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
	GPIO_configure(&GPIO_RF_TX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_RF_TX_ENABLE, 0);
	GPIO_configure(&GPIO_RF_RX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
}

/*******************************************************************/
void SKY13317_de_init(void) {
	// Set all pins to output low.
	SKY13317_set_channel(SKY13317_CHANNEL_NONE);
}

/*******************************************************************/
SKY13317_status_t SKY13317_set_channel(SKY13317_channel_t channel) {
	// Local variables.
	SKY13317_status_t status = SKY13317_SUCCESS;
	// Reset channels.
#ifdef HW1_0
	GPIO_write(&GPIO_RF_CHANNEL_A, 0);
	GPIO_write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
	GPIO_write(&GPIO_RF_TX_ENABLE, 0);
	GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
	// Select channel.
	switch (channel) {
	case SKY13317_CHANNEL_NONE:
		// Already done by previous reset.
		break;
#ifdef HW1_0
	case SKY13317_CHANNEL_RF1:
		GPIO_write(&GPIO_RF_CHANNEL_A, 1);
		break;
#endif
	case SKY13317_CHANNEL_RF2:
#ifdef HW1_0
		GPIO_write(&GPIO_RF_CHANNEL_A, 1);
		GPIO_write(&GPIO_RF_CHANNEL_B, 1);
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_RF_TX_ENABLE, 1);
#endif
		break;
	case SKY13317_CHANNEL_RF3:
#ifdef HW1_0
		GPIO_write(&GPIO_RF_CHANNEL_B, 1);
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_RF_RX_ENABLE, 1);
#endif
		break;
	default:
		status = SKY13317_ERROR_CHANNEL;
		break;
	}
	return status;
}
