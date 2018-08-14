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
#include "hwt.h"
#include "i2c.h"
#include "iwdg.h"
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
#ifdef HARDWARE_TIMER
#include "usart.h"
#endif

/*** SPSWS global structures ***/

// SPSWS state machine.
typedef enum {
	SPSWS_STATE_RESET_HANDLER,
	SPSWS_STATE_INIT,
	SPSWS_STATE_GPS,
	SPSWS_STATE_SENSORS,
	SPSWS_STATE_CONFIG_STATUS,
	SPSWS_STATE_WATCHDOG_RESET_HANDLER,
	SPSWS_STATE_SLEEP
} SPSWS_State;

// SPSWS context.
typedef struct {
	SPSWS_State spsws_state;
	unsigned char spsws_iwdg_reset;
	sfx_u16 spsws_mcu_vdd;
	sfx_s16 spsws_mcu_temperature;
} SPSWS_Context;

/*** SPSWS global variables ***/

SPSWS_Context spsws_ctx;

/*** SPSWS main function ***/

#if (defined INTERMITTENT_MODE) || (defined HARDWARE_TIMER)
/* MAIN FUNCTION FOR INTERMITTENT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

	// LED on.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	/* Init context */
	spsws_ctx.spsws_state = SPSWS_STATE_RESET_HANDLER;
	spsws_ctx.spsws_iwdg_reset = 0;

	/* Main loop */
	while(1) {

		/* Perform main state machine */
		switch (spsws_ctx.spsws_state) {

		/* Reset state */
		case SPSWS_STATE_RESET_HANDLER:
			// Enable power interface clock.
			RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
			// Check reset reason.
			if (((RCC -> CSR) & (0b1 << 29)) != 0) {
				if ((PWR -> CSR) & (0b1 << 1)) {
					// RESET = Watchdog reset while in stanby-mode -> go back to stand-by mode without starting IWDG.
					spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
				}
				else {
					// RESET = Watchdog reset during previous program execution -> software failure.
					spsws_ctx.spsws_iwdg_reset = 1;
					//IWDG_Init();
					spsws_ctx.spsws_state = SPSWS_STATE_INIT;
				}
			}
			else {
				if ((PWR -> CSR) & (0b1 << 1)) {
					if (HWT_Expired() == 1) {
						// RESET = Hardware timer wake-up.
						spsws_ctx.spsws_state = SPSWS_STATE_INIT;
					}
					else {
						// Program should never come here (wake-up from stand-by mode without watchdog and without hardware timer).
						spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
					}
					HWT_Init(1);
					spsws_ctx.spsws_state = SPSWS_STATE_INIT;
				}
				else {
					// RESET = Power-on reset (POR).
					HWT_Init(0);
					// Check hardware timer output.
					if (HWT_Expired() == 1) {
						spsws_ctx.spsws_state = SPSWS_STATE_INIT;
					}
					else {
						spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
					}
				}
			}
			break;

		/* Init modules and peripherals */
		case SPSWS_STATE_INIT:
			// Init clock.
			RCC_Init();
			RCC_SwitchToHsi16MHz();
#ifdef HARDWARE_TIMER
			// Send Sigfox frame at start-up.
			USART_Init();
			//USART_SendString("AT$SF=00\r\n");
#endif
			// Init time.
			TIM_TimeInit();
			// Init NVM.
			NVM_Init();
			// Compute next state.
			if (spsws_ctx.spsws_iwdg_reset == 1) {
				spsws_ctx.spsws_state = SPSWS_STATE_WATCHDOG_RESET_HANDLER;
			}
			else {
				spsws_ctx.spsws_state = SPSWS_STATE_GPS;
			}
			break;

		/* GPS and time management */
		case SPSWS_STATE_GPS:
			GPS_Processing();
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

		/* Handler when a watchdog reset is detected */
		case SPSWS_STATE_WATCHDOG_RESET_HANDLER:
			// TBC: end a specific Sigfox message to indicate watchdog reset occured.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* Sleep mode state */
		case SPSWS_STATE_SLEEP:
			// LED off.
			GPIOB -> ODR &= ~(0b1 << 4);
			// Enter standby mode.
			PWR_EnterStandbyMode();
			break;

		/* Unknown state */
		default:
			break;
		}
	}

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
