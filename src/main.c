/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "adc.h"
#ifdef CM_RTC
#include "exti.h"
#endif
#include "geoloc.h"
#include "gpio_reg.h"
#ifdef IM_HWT
#include "hwt.h"
#endif
#include "i2c.h"
#include "lpuart.h"
#include "mcu_api.h"
#include "nvm.h"
#include "pwr.h"
#include "pwr_reg.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "sigfox_types.h"
#include "tim.h"
#ifdef ATM
#include "usart.h"
#endif

/*** SPSWS global structures ***/

// SPSWS state machine.
typedef enum {
	SPSWS_STATE_POR, // State at power on reset.
	SPSWS_STATE_INIT, // State after every wake-up from standby mode.
	SPSWS_STATE_GEOLOC, // GEOLOC processing.
	SPSWS_STATE_SENSORS, // Sensors processing.
	SPSWS_STATE_CONFIG_STATUS, // Downlink management.
	SPSWS_STATE_SLEEP // Standby mode.
} SPSWS_State;

// SPSWS context.
typedef struct {
	SPSWS_State spsws_state;
	sfx_u16 spsws_mcu_vdd;
	sfx_s16 spsws_mcu_temperature;
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

	// Configure debug LED as output.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	/* Init context */
	spsws_ctx.spsws_state = SPSWS_STATE_POR;

	/* Main loop */
	while(1) {

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
			GEOLOC_Processing();
			spsws_ctx.spsws_state = SPSWS_STATE_SENSORS;
			break;

		/* Sensor management */
		case SPSWS_STATE_SENSORS:
			// Get MCU and temperature.
			MCU_API_get_voltage_temperature(&spsws_ctx.spsws_mcu_vdd, &spsws_ctx.spsws_mcu_vdd, &spsws_ctx.spsws_mcu_temperature);
			// Init I2C.
			/*I2C_Init();
			unsigned char command_byte[1] = {0x46};
			unsigned char rx[2];
			while(1) {
				I2C_SendBytes(I2C_TEMPERATURE_SENSOR_ADDRESS, command_byte, 1);
				//I2C_GetBytes(I2C_TEMPERATURE_SENSOR_ADDRESS, rx, 2);
				GPIOB -> ODR |= (0b1 << 4);
				TIM_TimeWaitMs(500);
				GPIOB -> ODR &= ~(0b1 << 4);
				TIM_TimeWaitMs(500);
			}*/
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

	// Wait for external components to start-up.
	TIM_TimeInit();
	TIM_TimeWaitMilliseconds(5000);

	// Send start message through Sigfox.
	//USART_Init();
	//USART_SendString((unsigned char*) "AT$SB=0\r\n");
	TIM_TimeWaitMilliseconds(7000);

	// Init HWT.
	HWT_Init(1);

	// Get GEOLOC timestamp and calibrate HWT.
	GEOLOC_Processing();

	// Send stats message through Sigfox.
	unsigned char gps_status_byte = 0;
	GEOLOC_GetStatusByte(&gps_status_byte);
	unsigned int hwt_absolute_error = 0;
	unsigned char hwt_feedback_value = 0;
	unsigned char hwt_feedback_direction = 0;
	HWT_GetParameters(&hwt_absolute_error, &hwt_feedback_value, &hwt_feedback_direction);
	unsigned int hwt_voltage_reference_mv = 0;
	ADC_Init();
	ADC_GetHwtVoltageReferenceMv(&hwt_voltage_reference_mv);
	ADC_Off();
	// Build AT command.
	unsigned char sigfox_uart_cmd[26] = {0};
	sigfox_uart_cmd[0] = 'A';
	sigfox_uart_cmd[1] = 'T';
	sigfox_uart_cmd[2] = '$';
	sigfox_uart_cmd[3] = 'S';
	sigfox_uart_cmd[4] = 'F';
	sigfox_uart_cmd[5] = '=';
	sigfox_uart_cmd[6] = HexaToAscii((gps_status_byte & 0xF0) >> 4);
	sigfox_uart_cmd[7] = HexaToAscii(gps_status_byte & 0x0F);
	sigfox_uart_cmd[8] = HexaToAscii((hwt_absolute_error & 0xF0000000) >> 28);
	sigfox_uart_cmd[9] = HexaToAscii((hwt_absolute_error & 0x0F000000) >> 24);
	sigfox_uart_cmd[10] = HexaToAscii((hwt_absolute_error & 0x00F00000) >> 20);
	sigfox_uart_cmd[11] = HexaToAscii((hwt_absolute_error & 0x000F0000) >> 16);
	sigfox_uart_cmd[12] = HexaToAscii((hwt_absolute_error & 0x0000F000) >> 12);
	sigfox_uart_cmd[13] = HexaToAscii((hwt_absolute_error & 0x00000F00) >> 8);
	sigfox_uart_cmd[14] = HexaToAscii((hwt_absolute_error & 0x000000F0) >> 4);
	sigfox_uart_cmd[15] = HexaToAscii(hwt_absolute_error & 0x0000000F);
	sigfox_uart_cmd[16] = HexaToAscii((hwt_feedback_value & 0xF0) >> 4);
	sigfox_uart_cmd[17] = HexaToAscii(hwt_feedback_value & 0x0F);
	sigfox_uart_cmd[18] = HexaToAscii((hwt_feedback_direction & 0xF0) >> 4);
	sigfox_uart_cmd[19] = HexaToAscii(hwt_feedback_direction & 0x0F);
	sigfox_uart_cmd[20] = HexaToAscii((hwt_voltage_reference_mv & 0x000F000) >> 12);
	sigfox_uart_cmd[21] = HexaToAscii((hwt_voltage_reference_mv & 0x0000F00) >> 8);
	sigfox_uart_cmd[22] = HexaToAscii((hwt_voltage_reference_mv & 0x00000F0) >> 4);
	sigfox_uart_cmd[23] = HexaToAscii(hwt_voltage_reference_mv & 0x000000F);
	sigfox_uart_cmd[24] = '\r';
	sigfox_uart_cmd[25] = '\n';
	//USART_SendString(sigfox_uart_cmd);
	TIM_TimeWaitMilliseconds(7000);
	//USART_Off();

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
	return 0;
}
#endif
