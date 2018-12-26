/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Registers.
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"
// Peripherals.
#include "adc.h"
#include "aes.h"
#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "i2c.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "spi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
// Components.
#include "dps310.h"
#include "max11136.h"
#include "max5495.h"
#include "neom8n.h"
#include "sht3x.h"
#include "si1133.h"
#include "sky13317.h"
#include "sx1232.h"
#include "sigfox_types.h"
#include "wind.h"
// Applicative.
#include "at.h"
#include "geoloc.h"
#include "hwt.h"
#include "mode.h"

/*** SPSWS global structures ***/

// SPSWS state machine.
typedef enum {
	SPSWS_STATE_RESET_CHECK,
	SPSWS_STATE_HOUR_CHECK,
	SPSWS_STATE_NVM_UPDATE,
	SPSWS_STATE_INIT,
	SPSWS_STATE_POR,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_SLEEP
} SPSWS_State;

// SPSWS context.
typedef struct {
	SPSWS_State spsws_state;
	Timestamp spsws_timestamp_from_geoloc;
	unsigned char spsws_timestamp_retrieved_from_geoloc;
	unsigned char spsws_downlink_requested;
	unsigned char spsws_status;
} SPSWS_Context;

/*** SPSWS global variables ***/

SPSWS_Context spsws_ctx;

/*** SPSWS main function ***/

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();

	/* Init clocks */
	RCC_Init();
	RCC_SwitchToTcxo16MHz();
	//RTC_Init();

	/* Init peripherals */
	// External interrupts.
	EXTI_Init();
	// Timers.
	TIM21_Init();
	TIM21_Enable();
	TIM22_Init();
	TIM22_Enable();
	// DMA.
	DMA1_Init();
	// Analog.
	ADC1_Init();
	// Communication interfaces.
	//LPUART1_Init();
	USART2_Init();
	I2C1_Init();
	SPI1_Init();
	// Hardware AES.
	AES_Init();

	/* Init components */
	MAX11136_Init();
	SX1232_Init();
	NEOM8N_Init();
	SKY13317_Init();
	WIND_Init();
	SHT3X_Init();
	DPS310_Init();
	SI1133_Init();

	/* Init applicative layers */
	AT_Init();

	/* POR blink */
	unsigned int k = 0;
	for (k=0 ; k<20 ; k++) {
		GPIO_Toggle(GPIO_LED);
		TIM22_WaitMilliseconds(50);
	}

	/* Init AT interface */
	USART2_PowerOn();
	USART2_Enable();

	/* Main loop */
	while (1) {
		AT_Task();
	}

	// RTC first calibration.
	/*Timestamp main_timestamp;
	LPUART_PowerOn();
	while (RTC_GetCalibrationStatus() == 0) {
		NEOM8N_ReturnCode neom8n_return_code = NEOM8N_GetTimestamp(&main_timestamp, 60);
		if (neom8n_return_code == NEOM8N_SUCCESS) {
			RTC_Calibrate(&main_timestamp);
		}
	}
	LPUART_PowerOff();*/

	return 0;
}
#endif
