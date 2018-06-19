/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "adc.h"
#ifdef CONTINUOUS_MODE
#include "exti.h"
#endif
#include "gpio_reg.h"
#include "gps.h"
#include "i2c.h"
#include "lpuart.h"
#include "mcu_api.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "sigfox_types.h"
#include "tim.h"

#ifdef INTERMITTENT_MODE
/* MAIN FUNCTION FOR INTERMITTENT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	// Init clock.
	RCC_Init();
	RCC_SwitchToMsi131kHz();

	// Init time.
	TIM_TimeInit();

	// Read and update all NVM variables.
	NVM_Init();

	// Configure PB4 as output.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	// Wait for NMEA data.
	GPIOB -> ODR |= (0b1 << 4); // LED on.
	unsigned char sigfox_gps_data[11] = {0};
	while (GPS_Processing(sigfox_gps_data, 11) == 0);

/*	// Init I2C.
	I2C_Init();

	unsigned char command_byte[1] = {0x46};
	unsigned char rx[2];
	while(1) {
		I2C_SendBytes(I2C_TEMPERATURE_SENSOR_ADDRESS, command_byte, 1);
		//I2C_GetBytes(I2C_TEMPERATURE_SENSOR_ADDRESS, rx, 2);
		GPIOB -> ODR |= (0b1 << 4);
		TIM_TimeWaitMs(500);
		GPIOB -> ODR &= ~(0b1 << 4);
		TIM_TimeWaitMs(500);
	}
*/

	// Switch to HSI for ADC.
	RCC_SwitchToHsi16MHz();

	// Init ADC.
	ADC_Init();

	// Get MCU and temperature.
	sfx_u16 mcu_vdd;
	sfx_s16 mcu_temperature;
	MCU_API_get_voltage_temperature(&mcu_vdd, &mcu_vdd, &mcu_temperature);

	// LED off.
	GPIOB -> ODR &= ~(0b1 << 4);

	// Enter standby mode.
	PWR_EnterStandbyMode();

	return 0;
}
#endif

#ifdef CONTINUOUS_MODE
/* MAIN FUNCTION FOR CONTINUOUS MODE.
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
