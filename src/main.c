/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Registers
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
#include "iwdg.h"
#include "lptim.h"
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
#include "dlk.h"
#include "geoloc.h"
#ifdef IM_HWT
#include "hwt.h"
#endif
#include "mode.h"
#include "monitoring.h"
#include "sigfox_api.h"
#include "weather.h"

/*** SPSWS structures ***/

typedef enum {
	SPSWS_STATE_RESET,
	SPSWS_STATE_POR,
	SPSWS_STATE_HOUR_CHECK,
	SPSWS_STATE_NVM_UPDATE,
	SPSWS_STATE_INIT,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_SLEEP
} SPSWS_State;

typedef enum {
	SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX,
	SPSWS_STATUS_BYTE_TIME_REFERENCE_BIT_IDX,
	SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_FIRST_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX,
} SPSWS_StatusByteBitIndex;

typedef struct {
	SPSWS_State spsws_state;
	unsigned char spsws_status_byte;
	Timestamp spsws_current_timestamp;
	Timestamp spsws_previous_wake_up_timestamp;
	unsigned char spsws_hours_count;
	unsigned char spsws_weather_data_period_hours;
	unsigned char spsws_day_count;
	unsigned char spsws_geoloc_period_days;
	DLK_Parameters spsws_dlk_parameters;
	unsigned char spsws_timestamp_retrieved_from_geoloc;
	MONITORING_Data spsws_monitoring_data;
	WEATHER_Data spsws_weather_data;
} SPSWS_Context;

/*** SPSWS global variables ***/

static SPSWS_Context spsws_ctx;

/*** SPSWS main function ***/

#ifdef IM_RTC
/* MAIN FUNCTION FOR IM_RTC MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Reset deep sleep and wake-up flags on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
	PWR -> CR |= (0b1 << 2); // CWUF='1'.

	/* Init context */
	spsws_ctx.spsws_state = SPSWS_STATE_RESET;
	spsws_ctx.spsws_state = SPSWS_STATE_NVM_UPDATE; // !!! DEBUG !!!
	spsws_ctx.spsws_status_byte = 0; // IM_RTC = '00'.
	spsws_ctx.spsws_timestamp_retrieved_from_geoloc = 0;

	/* Local variables */
	unsigned char nvm_byte = 0;
	NEOM8N_ReturnCode neom8n_return_code;
	sfx_rc_t spsws_sigfox_rc = (sfx_rc_t) SPSWS_SIGFOX_RC;
	sfx_error_t sfx_error = SFX_ERR_NONE;

	/* Main loop */
	while (1) {

		/* Perform state machine */
		switch (spsws_ctx.spsws_state) {

		/* RESET */
		case SPSWS_STATE_RESET:
			// Check reset reason.
			if (((RCC -> CSR) & (0b1 << 29)) != 0) {
				// Watchdog reset: directly enter standby mode.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			else {
				if (((RCC -> CSR) & (0b1 << 27)) != 0) {
					// Power-on reset: directly go to INIT state.
					spsws_ctx.spsws_state = SPSWS_STATE_INIT;
				}
				else {
					// RTC wake-up: standard flow.
					spsws_ctx.spsws_state = SPSWS_STATE_HOUR_CHECK;
				}
			}
			break;

		/* HOUR CHECK */
		case SPSWS_STATE_HOUR_CHECK:
			// Retrieve current timestamp from RTC.
			RTC_GetTimestamp(&spsws_ctx.spsws_current_timestamp);
			// Retrieve previous wake-up timestamp from NVM.
			NVM_Enable();
			NVM_ReadByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), &nvm_byte);
			spsws_ctx.spsws_previous_wake_up_timestamp.year = (nvm_byte << 8);
			NVM_ReadByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), &nvm_byte);
			spsws_ctx.spsws_previous_wake_up_timestamp.year |= nvm_byte;
			NVM_ReadByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.month);
			NVM_ReadByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.date);
			NVM_ReadByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.hours);
			// Check timestamp are differents (avoiding false wake-up due to RTC recalibration).
			if ((spsws_ctx.spsws_current_timestamp.year != spsws_ctx.spsws_previous_wake_up_timestamp.year) ||
				(spsws_ctx.spsws_current_timestamp.month != spsws_ctx.spsws_previous_wake_up_timestamp.month) ||
				(spsws_ctx.spsws_current_timestamp.date != spsws_ctx.spsws_previous_wake_up_timestamp.date) ||
				(spsws_ctx.spsws_current_timestamp.hours != spsws_ctx.spsws_previous_wake_up_timestamp.hours)) {
				// Valid wake-up;
				spsws_ctx.spsws_state = SPSWS_STATE_NVM_UPDATE;
			}
			else {
				// False wake-up: directly enter sleep mode.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			break;

		/* NVM UPDATE */
		case SPSWS_STATE_NVM_UPDATE:
			// Read current configuration.
			DLK_Read(&spsws_ctx.spsws_dlk_parameters);
			// Update previous wake-up timestamp.
			NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), ((spsws_ctx.spsws_current_timestamp.year & 0xFF00) >> 8));
			NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), ((spsws_ctx.spsws_current_timestamp.year & 0x00FF) >> 0));
			NVM_WriteByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.month);
			NVM_WriteByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.date);
			NVM_WriteByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.hours);
			// Update hours count for weather data period management.
			NVM_ReadByte(NVM_HOURS_COUNT_ADDRESS_OFFSET, &spsws_ctx.spsws_hours_count);
			spsws_ctx.spsws_hours_count++;
			if (spsws_ctx.spsws_hours_count >= spsws_ctx.spsws_dlk_parameters.dlk_weather_data_period_hours) {
				// Period reached, reset counter.
				NVM_WriteByte(NVM_HOURS_COUNT_ADDRESS_OFFSET, 0);
			}
			else {
				// Update value.
				NVM_WriteByte(NVM_HOURS_COUNT_ADDRESS_OFFSET, spsws_ctx.spsws_hours_count);
			}
			// Check if day changed.
			if ((spsws_ctx.spsws_current_timestamp.year != spsws_ctx.spsws_previous_wake_up_timestamp.year) ||
				(spsws_ctx.spsws_current_timestamp.month != spsws_ctx.spsws_previous_wake_up_timestamp.month) ||
				(spsws_ctx.spsws_current_timestamp.date != spsws_ctx.spsws_previous_wake_up_timestamp.date)) {
				// Update day count for geolocation period management.
				NVM_ReadByte(NVM_DAY_COUNT_ADDRESS_OFFSET, &spsws_ctx.spsws_day_count);
				spsws_ctx.spsws_day_count++;
				if (spsws_ctx.spsws_day_count >= spsws_ctx.spsws_dlk_parameters.dlk_geoloc_period_days) {
					// Period reached, reset counter.
					NVM_WriteByte(NVM_DAY_COUNT_ADDRESS_OFFSET, 0);
				}
				else {
					NVM_WriteByte(NVM_DAY_COUNT_ADDRESS_OFFSET, spsws_ctx.spsws_day_count);
				}
				// Reset daily RTC calibration and downlink flags.
				NVM_ReadByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, &spsws_ctx.spsws_status_byte);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
				NVM_WriteByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, spsws_ctx.spsws_status_byte);
			}
			// Go to INIT state.
			spsws_ctx.spsws_state = SPSWS_STATE_INIT;
			break;

		/* INIT */
		case SPSWS_STATE_INIT:
			// Init GPIOs (required for clock tree configuration).
			GPIO_Init();
			GPIO_Write(GPIO_LED, 1);
			// Init clocks.
			RCC_Init();
			// High speed oscillator.
			spsws_ctx.spsws_status_byte |= (RCC_SwitchToTcxo16MHz() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
				while (RCC_SwitchToInternal16MHz() == 0);
			}
			// Low speed oscillators.
			RCC_EnableInternal32kHz();
			spsws_ctx.spsws_status_byte |= (RCC_EnableCrystal32kHz() << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX);
			// Watchdog.
			//IWDG_Init();
			// RTC (initialized at power-on only).
			if (((RCC -> CSR) & (0b1 << 27)) != 0) {
				RTC_Init(spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX));
			}
			// Timers.
			TIM21_Init();
			TIM21_Enable();
			TIM22_Init();
			TIM22_Enable();
			LPTIM1_Init();
			// DMA.
			DMA1_Init();
			// Analog.
			ADC1_Init();
			// Communication interfaces.
			LPUART1_Init();
			I2C1_Init();
			SPI1_Init();
#ifdef HW2_0
			SPI2_Init();
#endif
			// Hardware AES.
			AES_Init();
			// Init components.
			SX1232_Init();
#ifdef HW2_0
			SX1232_Tcxo(1);
#endif
			SKY13317_Init();
			NEOM8N_Init();
			MAX11136_Init();
			SHT3X_Init();
			DPS310_Init();
			SI1133_Init();
			// Compute next state.
			if (((RCC -> CSR) & (0b1 << 27)) != 0) {
				spsws_ctx.spsws_state = SPSWS_STATE_POR;
			}
			else {
				spsws_ctx.spsws_state = SPSWS_STATE_MEASURE;
			}
			break;

		/* MEASURE */
		case SPSWS_STATE_MEASURE:
			// Send bit.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_bit(0, (void*) 0, 2, 0);
			}
			SIGFOX_API_close();
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* POR */
		case SPSWS_STATE_POR:
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			}
			SIGFOX_API_close();
			// Perform first RTC calibration.
			spsws_ctx.spsws_state = SPSWS_STATE_RTC_CALIBRATION;
			break;

		/* RTC CALIBRATION */
		case SPSWS_STATE_RTC_CALIBRATION:
			// Use GPS module if timestamp was not retrieved yet by the geolocation.
			if (spsws_ctx.spsws_timestamp_retrieved_from_geoloc == 0) {
				LPUART1_PowerOn();
				neom8n_return_code = NEOM8N_GetTimestamp(&spsws_ctx.spsws_current_timestamp, spsws_ctx.spsws_dlk_parameters.dlk_gps_timeout_seconds);
				LPUART1_PowerOff();
			}
			// Calibrate RTC id a valid timestamp is available.
			if ((spsws_ctx.spsws_timestamp_retrieved_from_geoloc != 0) || (neom8n_return_code == NEOM8N_SUCCESS)) {
				RTC_Calibrate(&spsws_ctx.spsws_current_timestamp);
			}
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* SLEEP */
		case SPSWS_STATE_SLEEP:
			// Switch 32MHz TCXO off.
#ifdef HW2_0
			SX1232_Tcxo(0);
#endif
			// Enable RTC interrupt.
			RTC_EnableInterrupt();
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23);
			// Switch 16MHz TCXO off.
			RCC_SwitchToInternal16MHz();
			// Enter standby mode.
			PWR_EnterStandbyMode();
			break;

		/* UNKNOWN STATE */
		default:
			break;
		}
	}

	return 0;
}
#endif

#ifdef IM_HWT
int main (void) {
	return 0;
}
#endif

#ifdef CM_RTC
int main (void) {
	return 0;
}
#endif

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init context */
	spsws_ctx.spsws_status_byte = 0;

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();

	/* Init clocks */
	RCC_Init();
	// High speed oscillator.
	spsws_ctx.spsws_status_byte |= (RCC_SwitchToTcxo16MHz() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
	if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
		while (RCC_SwitchToInternal16MHz() == 0);
	}
	// Low speed oscillators.
	RCC_EnableInternal32kHz();
	spsws_ctx.spsws_status_byte |= (RCC_EnableCrystal32kHz() << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX);

	/* Init watchdog */
	IWDG_Init();

	/* Init peripherals */
	// Real time clock.
	RTC_Init(spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX));
	// External interrupts.
	EXTI_Init();
	// Timers.
	TIM21_Init();
	TIM21_Enable();
	TIM22_Init();
	TIM22_Enable();
	LPTIM1_Init();
	// DMA.
	DMA1_Init();
	// Analog.
	ADC1_Init();
	// Communication interfaces.
	LPUART1_Init();
#ifdef HW1_0
	USART2_Init();
#endif
#ifdef HW2_0
	USART1_Init();
#endif
	I2C1_Init();
	SPI1_Init();
#ifdef HW2_0
	SPI2_Init();
#endif
#ifdef AT_COMMANDS_SIGFOX
	// Hardware AES.
	AES_Init();
#endif

	/* Init components */
#ifdef AT_COMMANDS_SIGFOX
	SX1232_Init();
#ifdef HW2_0
	SX1232_Tcxo(1);
#endif
	SKY13317_Init();
#endif
#ifdef AT_COMMANDS_GPS
	NEOM8N_Init();
#endif
#ifdef AT_COMMANDS_SENSORS
	MAX11136_Init();
	WIND_Init();
	SHT3X_Init();
	DPS310_Init();
	SI1133_Init();
#endif

	/* Init applicative layers */
	AT_Init();

	/* Main loop */
	while (1) {

		/* Perform tasks */
		AT_Task();
		if (RTC_GetIrqStatus() != 0) {
			USARTx_SendString("RTC\n");
			RTC_ClearIrqStatus();
		}

		/* Reload watchdog */
		IWDG_Reload();
	}

	return 0;
}
#endif
