/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "adc.h"
#include "at.h"
#include "exti.h"
#include "geoloc.h"
#include "gpio_reg.h"
#include "hwt.h"
#include "i2c.h"
#include "lpuart.h"
#include "max11136.h"
#include "max5495.h"
#include "mcu_api.h"
#include "neom8n.h"
#include "nvm.h"
#include "pwr.h"
#include "pwr_reg.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "sht3x.h"
#include "sigfox_types.h"
#include "spi.h"
#include "sx1232.h"
#include "sx1232_reg.h"
#include "rtc.h"
#include "tim.h"
#include "ultimeter.h"
#include "usart.h"

/*** SPSWS global structures ***/

// SPSWS state machine.
typedef enum {
	SPSWS_STATE_POR,
	SPSWS_STATE_INIT,
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_WEATHER_DATA,
	SPSWS_STATE_CONFIG_STATUS,
	SPSWS_STATE_SLEEP
} SPSWS_State;

// SPSWS context.
typedef struct {
	SPSWS_State spsws_state;
	sfx_u16 spsws_mcu_vdd;
	sfx_s16 spsws_mcu_temperature;
	GPS_TimestampData spsws_timestamp_from_geoloc;
	unsigned char spsws_timestamp_retrieved_from_geoloc;
} SPSWS_Context;

/*** SPSWS global variables ***/

SPSWS_Context spsws_ctx;

/*** SPSWS main function ***/

#ifdef IM_RTC
/* MAIN FUNCTION FOR INTERMITTENT MODE USING RTC.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	/* Init context */
	spsws_ctx.spsws_state = SPSWS_STATE_POR;

	/* Main loop */
	while (1) {

		/* Perform main state machine */
		switch (spsws_ctx.spsws_state) {

		/* Power on reset state */
		case SPSWS_STATE_POR:
			// Init RTC.

			// Wait for RTC wake-up.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* Init peripherals */
		case SPSWS_STATE_INIT:
			// Init clock.
			RCC_Init();
			RCC_SwitchToHsi16MHz();
			// Init time.
			TIM_TimeInit();
			// Init NVM.
			NVM_Init();
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_GEOLOC;
			break;

		/* GEOLOC and time management */
		case SPSWS_STATE_GEOLOC:
			GEOLOC_Process(&spsws_ctx.spsws_timestamp_from_geoloc, &spsws_ctx.spsws_timestamp_retrieved_from_geoloc);
			spsws_ctx.spsws_state = SPSWS_STATE_WEATHER_DATA;
			break;

		/* Sensor management */
		case SPSWS_STATE_WEATHER_DATA:
			// TBC.
			spsws_ctx.spsws_state = SPSWS_STATE_CONFIG_STATUS;
			break;

		/* Downlink management */
		case SPSWS_STATE_CONFIG_STATUS:
			// TBC.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* Sleep mode state */
		case SPSWS_STATE_SLEEP:
			// LED off.
			GPIOB -> ODR &= ~(0b1 << 4);
			// Enter standby mode.
			PWR_EnterStandbyMode();
			// Wake-up.
			spsws_ctx.spsws_state = SPSWS_STATE_INIT;
			break;

		/* Unknown state */
		default:
			break;
		}
	}

	return 0;
}
#endif

#ifdef CM_RTC
/* MAIN FUNCTION FOR CONTINUOUS MODE USING RTC.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	// Init clock.
	RCC_Init();
	RCC_SwitchToHsi16MHz();

	// Init time.
	TIM_TimeInit();

	// Configure PB4 as output.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	// LED off.
	GPIOB -> ODR &= ~(0b1 << 4);

	// Configure external interrupt.
	EXTI_Init();

	while(1);

	return 0;
}
#endif

#ifdef IM_HWT
/* RETURN THE ASCII CODE OF A GIVEN HEXADIMAL VALUE.
 * @param value:	Hexadecimal value to convert.
 * @return c:		Correspoding ASCII code.
 */
unsigned char HexaToAscii(unsigned char value) {
	unsigned char c = 0;
	if ((value >= 0) && (value <= 9)) {
		c = value + '0';
	}
	else {
		if ((value >= 10) && (value <= 15)) {
			c = value + 'A' - 10;
		}
	}
	return c;
}

/* MAIN FUNCTION FOR INTERMITTENT MODE USING HARDWARE TIMER.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	// Reset deep sleep flag on wake-up
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	// Init clock.
	RCC_Init();
	RCC_SwitchToHsi16MHz();

	// Configure wake-up pin as input.
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b11 << 0); // Reset bits 0-1.

	// Init time.
	TIM_TimeInit();

	// Debug LED on.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);
	GPIOB -> ODR |= (0b1 << 4);

	// Reset timer.
	HWT_Reset();

	// Wait for external components to start-up.
	TIM_TimeInit();
	TIM_TimeWaitMilliseconds(5000);

	// Send start message through Sigfox.
	// TBD.

	// Calibrate timer.
	HWT_Process(1, 0, spsws_ctx.spsws_timestamp_from_geoloc);

	// Check if HWT has not expired yet (effective duration < MCU run duration).
	if (HWT_Expired() == 1) {
		// TBD: trigger software reset.
	}
	else {
		// LED off.
		GPIOB -> ODR &= ~(0b1 << 4);
		// Enter standby mode.
		PWR_EnterStandbyMode();
	}

	return 0;
}
#endif

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	// Init clocks.
	RCC_Init();
	RCC_SwitchToInternal16MHz();
	RTC_Init();

	// Init peripherals.
	EXTI_Init();
	TIM21_Init();
	TIM22_Init();
	USART_Init();
	I2C_Init();
	NVM_Init();

	// Init applicative layers.
	AT_Init();

	// Configure debug LED as output (PA2)
	RCC -> IOPENR |= (0b11 << 0);
	GPIOA -> MODER &= ~(0b11 << 4); // Reset bits 8-9.
	GPIOA -> MODER |= (0b01 << 4);

	// POR blink.
	unsigned int i = 0;
	unsigned int k = 0;
	for (k=0 ; k<5 ; k++) {
		GPIOA -> ODR |= (0b1 << 2);
		for (i=0 ; i<50000 ; i++);
		GPIOA -> ODR &= ~(0b1 << 2);
		for (i=0 ; i<50000 ; i++);
	}
	TIM22_WaitMilliseconds(100);

	//SX1232_Init();
	//MAX11136_Init();
	//MAX5495_Init();
	//SPI_PowerOn();

	/*while (1) {
		USART_SendString("Test UART : ");
		USART_SendValue(0, USART_Binary);
		USART_SendString(" ");
		USART_SendValue(0, USART_Hexadecimal);
		USART_SendString(" ");
		USART_SendValue(0, USART_Decimal);
		USART_SendString(" ");
		USART_SendValue(0, USART_ASCII);
		USART_SendString("\n");
		TIM_TimeWaitMilliseconds(1000);
		GPIOA -> ODR |= (0b1 << 2);
		for (i=0 ; i<50000 ; i++);
		GPIOA -> ODR &= ~(0b1 << 2);
		for (i=0 ; i<50000 ; i++);
	}*/

	/*while (1) {
		// Retrieve GPS position and timestamp.
		GEOLOC_Process(&spsws_ctx.spsws_timestamp_from_geoloc, &spsws_ctx.spsws_timestamp_retrieved_from_geoloc);
		// Blink LED.
		GPIOA -> ODR |= (0b1 << 2);
		for (i=0 ; i<50000 ; i++);
		GPIOA -> ODR &= ~(0b1 << 2);
		for (i=0 ; i<50000 ; i++);
	}*/

	/*
	//unsigned char tmp_sht = 0;
	//unsigned char hum_sht = 0;
	while (1) {
		//SHT3X_ReadTemperatureHumidity(&tmp_sht, &hum_sht);
		MAX11136_ConvertAllChannels();
		TIM_TimeWaitMilliseconds(1000);
		GPIOA -> ODR |= (0b1 << 2);
		for (i=0 ; i<50000 ; i++);
		GPIOA -> ODR &= ~(0b1 << 2);
		for (i=0 ; i<50000 ; i++);
	}*/

	/*unsigned char sx1232_version = 0;
	unsigned char sx1232_reg_tcxo = 0;
	SX1232_Start();
	while (1) {
		SX1232_ReadRegister(SX1232_REG_TCXO, &sx1232_reg_tcxo);
		SX1232_ReadRegister(SX1232_REG_VERSION, &sx1232_version);
		USART_SendString("SX1232 revision = ");
		USART_SendValue(sx1232_version, USART_Hexadecimal);
		USART_SendString("\n");
		USART_SendString("SX1232 reg tcxo = ");
		USART_SendValue(sx1232_reg_tcxo, USART_Hexadecimal);
		USART_SendString("\n");
		TIM_TimeWaitMilliseconds(1000);
		GPIOA -> ODR |= (0b1 << 2);
		for (i=0 ; i<50000 ; i++);
		GPIOA -> ODR &= ~(0b1 << 2);
		for (i=0 ; i<50000 ; i++);
	}*/

	ULTIMETER_Init();
	ULTIMETER_StartContinuousMeasure();
	while (1) {
		AT_Task();
	}

	return 0;
}
#endif
