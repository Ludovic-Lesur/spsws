/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Registers
#include "lptim_reg.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "tim_reg.h"
// Peripherals.
#include "adc.h"
#include "aes.h"
#include "dma.h"
#include "exti.h"
#include "flash.h"
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

/*** SPSWS macros ***/

// GPS timeout when calibrating RTC.
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS	180
#define SPSWS_NUMBER_OF_HOURS_PER_DAY			24
#define SPSWS_AFTERNOON_HOUR_THRESHOLD			12

/*** SPSWS structures ***/

typedef enum {
	SPSWS_STATE_RESET,
	SPSWS_STATE_NVM_WAKE_UP_UPDATE,
	SPSWS_STATE_HOUR_CHECK,
	SPSWS_STATE_INIT,
	SPSWS_STATE_POR,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_NVM_POR_UPDATE,
	SPSWS_STATE_SLEEP
} SPSWS_State;

typedef enum {
	SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX,
	SPSWS_STATUS_BYTE_TIME_REFERENCE_BIT_IDX,
	SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX,
	SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_FIRST_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX,
} SPSWS_StatusByteBitIndex;

typedef struct {
	unsigned char spsws_por_flag;
	unsigned char spsws_hour_changed_flag;
	unsigned char spsws_day_changed_flag;
	unsigned char spsws_is_afternoon_flag;
	unsigned int spsws_lsi_frequency_hz;
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
	unsigned char spsws_sfx_uplink_data[SFX_UPLINK_DATA_MAX_SIZE_BYTES];
	unsigned char spsws_sfx_downlink_data[SFX_DOWNLINK_DATA_SIZE_BYTES];
} SPSWS_Context;

/*** SPSWS global variables ***/

static SPSWS_Context spsws_ctx;

/*** SPSWS local functions ***/

/* MAKE THE DEBUG BLINK.
 * @param number_of_blinks:	Number of blinks.
 * @return:					None.
 */
void SPSWS_BlinkLed(unsigned char number_of_blinks) {
	unsigned char j = 0;
	unsigned int k = 0;
	for (j=0 ; j<number_of_blinks ; j++) {
		GPIO_Write(GPIO_LED, 1);
		for (k=0 ; k<10000 ; k++);
		GPIO_Write(GPIO_LED, 0);
		for (k=0 ; k<10000 ; k++);
	}
}

/* CHECK IF HOUR OR DATE AS CHANGED SINCE PREVIOUS WAKE-UP AND UPDATE AFTERNOON FLAG.
 * @param:	None.
 * @return:	None.
 */
void SPSWS_UpdateTimestampFlags(void) {
	// Retrieve current timestamp from RTC.
	RTC_GetTimestamp(&spsws_ctx.spsws_current_timestamp);
	// Retrieve previous wake-up timestamp from NVM.
	unsigned char nvm_byte = 0;
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
		(spsws_ctx.spsws_current_timestamp.date != spsws_ctx.spsws_previous_wake_up_timestamp.date)) {
		// Day and thus hour has changed.
		spsws_ctx.spsws_day_changed_flag = 1;
		spsws_ctx.spsws_hour_changed_flag = 1;
	}
	if (spsws_ctx.spsws_current_timestamp.hours != spsws_ctx.spsws_previous_wake_up_timestamp.hours) {
		// Hour as changed.
		spsws_ctx.spsws_hour_changed_flag = 1;
	}
	// Check if we are in afternoon (to trigger geolocation and downlink).
	signed char local_hour = (spsws_ctx.spsws_current_timestamp.hours + spsws_ctx.spsws_dlk_parameters.dlk_local_utc_offset) % SPSWS_NUMBER_OF_HOURS_PER_DAY;
	if (local_hour < 0) {
		local_hour += SPSWS_NUMBER_OF_HOURS_PER_DAY;
	}
	if (local_hour >= SPSWS_AFTERNOON_HOUR_THRESHOLD) {
		spsws_ctx.spsws_is_afternoon_flag = 1;
	}
}

/* UPDATE PREVIOUS WAKE-UP TIMESTAMP IN NVM.
 * @param:	None.
 * @return:	None.
 */
void SPSWS_UpdatePwut(void) {
	// Retrieve current timestamp from RTC.
	RTC_GetTimestamp(&spsws_ctx.spsws_current_timestamp);
	// Update previous wake-up timestamp.
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), ((spsws_ctx.spsws_current_timestamp.year & 0xFF00) >> 8));
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), ((spsws_ctx.spsws_current_timestamp.year & 0x00FF) >> 0));
	NVM_WriteByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.month);
	NVM_WriteByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.date);
	NVM_WriteByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.hours);
}

/* COMPUTE EFFECTIVE LSI OSCILLATOR FREQUENCY.
 * @param:	None.
 * @return:	None.
 */
void SPSWS_ComputeLsiFrequency(void) {

	/* Init and start timers */
	LPTIM1_Init(1);
	TIM21_Init();
	LPTIM1_Start();
	TIM21_Start();

	/* Wait 1 second */
	while (((TIM21 -> SR) & (0b1 << 0)) == 0); // Wait for first overflow.
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').

	/* Get LSI frequency and stop timers */
	spsws_ctx.spsws_lsi_frequency_hz = (LPTIM1 -> CNT);
	LPTIM1_Stop();
	TIM21_Stop();

	/* Check value */
	if (spsws_ctx.spsws_lsi_frequency_hz == 0) {
		spsws_ctx.spsws_lsi_frequency_hz = 38000;
	}
}

/*** SPSWS main function ***/

#ifdef IM_RTC
/* MAIN FUNCTION FOR IM_RTC MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init memory */
	FLASH_Init();
	NVM_Enable();

	// Init GPIOs (required for clock tree configuration).
	GPIO_Init();
	GPIO_Write(GPIO_LED, 1);

	/* Init context */
	spsws_ctx.spsws_por_flag = 0;
	spsws_ctx.spsws_hour_changed_flag = 0;
	spsws_ctx.spsws_day_changed_flag = 0;
	spsws_ctx.spsws_is_afternoon_flag = 0;
	spsws_ctx.spsws_state = SPSWS_STATE_RESET;
	NVM_ReadByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, &spsws_ctx.spsws_status_byte);
	spsws_ctx.spsws_timestamp_retrieved_from_geoloc = 0;
	// IM_RTC = '00'.
	spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX);
	spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_TIME_REFERENCE_BIT_IDX);

	/* Local variables */
	NEOM8N_ReturnCode neom8n_return_code;
	sfx_rc_t spsws_sigfox_rc = (sfx_rc_t) SPSWS_SIGFOX_RC;
	sfx_error_t sfx_error = SFX_ERR_NONE;
	unsigned int max11136_result_12bits = 0;

	/* Main loop */
	while (1) {

		/* Perform state machine */
		switch (spsws_ctx.spsws_state) {

		/* RESET */
		case SPSWS_STATE_RESET:
			// Check reset reason.
			if (((RCC -> CSR) & (0b1 << 29)) != 0) {
				// IWDG reset: directly enter standby mode.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			else {
				if (((RCC -> CSR) & (0b111 << 26)) != 0) {
					// POR, NRST or SW reset: directly go to INIT state.
					spsws_ctx.spsws_por_flag = 1;
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
			// Retrieve current configuration (done here to read local UTC offset).
			DLK_Read(&spsws_ctx.spsws_dlk_parameters);
			// Update flags.
			SPSWS_UpdateTimestampFlags();
			// Check flag.
			if (spsws_ctx.spsws_hour_changed_flag == 0) {
				// False detection due to RTC recalibration.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			else {
				// Valid wake-up.
				spsws_ctx.spsws_state = SPSWS_STATE_NVM_WAKE_UP_UPDATE;
			}
			spsws_ctx.spsws_state = SPSWS_STATE_NVM_WAKE_UP_UPDATE; // !!! DEBUG !!!
			break;

		/* NVM WAKE UP UPDATE */
		case SPSWS_STATE_NVM_WAKE_UP_UPDATE:
			// Update previous wake-up timestamp.
			SPSWS_UpdatePwut();
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
			if (spsws_ctx.spsws_day_changed_flag != 0) {
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
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
			}
			// Go to INIT state.
			spsws_ctx.spsws_state = SPSWS_STATE_INIT;
			break;

		/* INIT */
		case SPSWS_STATE_INIT:
			// Init clocks.
			RCC_Init();
			// Low speed oscillators (only at POR).
			if (spsws_ctx.spsws_por_flag != 0) {
				// LSI.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableInternal32kHz() << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				// LSE.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableCrystal32kHz() << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			}
			// High speed oscillator.
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			spsws_ctx.spsws_status_byte |= (RCC_SwitchToTcxo16MHz() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
				while (RCC_SwitchToInternal16MHz() == 0);
			}
			// RTC (only at POR).
			if (spsws_ctx.spsws_por_flag != 0) {
				SPSWS_ComputeLsiFrequency();
				RTC_Init((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX)), spsws_ctx.spsws_lsi_frequency_hz);
			}
			// Watchdog.
			//IWDG_Init();
			//IWDG_Reload();
			// Timers.
			TIM21_Init();
			TIM22_Init();
			TIM21_Start();
			TIM22_Start();
			LPTIM1_Init(0);
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
			// Reload watchdog.
			IWDG_Reload();
			// Compute next state.
			if (spsws_ctx.spsws_por_flag == 0) {
				spsws_ctx.spsws_state = SPSWS_STATE_MEASURE;
			}
			else {
				spsws_ctx.spsws_state = SPSWS_STATE_POR;
			}
			break;

		/* MEASURE */
		case SPSWS_STATE_MEASURE:
			// Retrieve internal ADC data.
			ADC1_PerformMeasurements();
			ADC1_GetMcuTemperature(&spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_temperature_degrees);
			ADC1_GetMcuSupplyVoltage(&spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv);
			// Retrieve external ADC data.
#ifdef HW1_0
			SPI1_PowerOn();
#endif
#ifdef HW2_0
			SPI2_PowerOn();
#endif
			MAX11136_PerformMeasurements();
#ifdef HW1_0
			SPI1_PowerOff();
#endif
#ifdef HW2_0
			SPI2_PowerOff();
#endif
			MAX11136_GetChannel(MAX11136_CHANNEL_SOLAR_CELL, &max11136_result_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_solar_cell_voltage_mv = (spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv * max11136_result_12bits * 269) / (MAX11136_FULL_SCALE * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_SUPERCAP, &max11136_result_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_supercap_voltage_mv = (spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv * max11136_result_12bits * 269) / (MAX11136_FULL_SCALE * 34);
			// Retrieve weather sensors data.
			I2C1_PowerOn();
			SHT3X_PerformMeasurements(SHT3X_INTERNAL_I2C_ADDRESS);
			I2C1_PowerOff();
			SHT3X_GetTemperature(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_temperature_degrees);
			SHT3X_GetHumidity(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_humidity_percent);
			// Read status byte.
			spsws_ctx.spsws_monitoring_data.monitoring_data_status_byte = spsws_ctx.spsws_status_byte;
			// Reload watchdog.
			IWDG_Reload();
			// Go to MONITORING state.
			spsws_ctx.spsws_state = SPSWS_STATE_MONITORING;
			break;

		/* MONITORING */
		case SPSWS_STATE_MONITORING:
			// Build Sigfox frame.
			MONITORING_BuildSigfoxData(&spsws_ctx.spsws_monitoring_data, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sfx_uplink_data, MONITORING_SIGFOX_DATA_LENGTH, spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			// Reload watchdog.
			IWDG_Reload();
			// Compute next state.
			if (((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX)) == 0) && (spsws_ctx.spsws_is_afternoon_flag != 0)) {
				// Perform RTC calibration.
				spsws_ctx.spsws_state = SPSWS_STATE_RTC_CALIBRATION;
			}
			else {
				// Enter standby mode.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			break;

		/* POR */
		case SPSWS_STATE_POR:
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			}
			SIGFOX_API_close();
			// Reset RTC calibration flags.
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX);
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX);
			// Reload watchdog.
			IWDG_Reload();
			// Perform first RTC calibration.
			spsws_ctx.spsws_state = SPSWS_STATE_RTC_CALIBRATION;
			break;

		/* RTC CALIBRATION */
		case SPSWS_STATE_RTC_CALIBRATION:
			// Use GPS module if timestamp was not retrieved yet by the geolocation.
			if (spsws_ctx.spsws_timestamp_retrieved_from_geoloc == 0) {
				LPUART1_PowerOn();
				neom8n_return_code = NEOM8N_GetTimestamp(&spsws_ctx.spsws_current_timestamp, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS);
				LPUART1_PowerOff();
			}
			// Calibrate RTC if a valid timestamp is available.
			if ((spsws_ctx.spsws_timestamp_retrieved_from_geoloc != 0) || (neom8n_return_code == NEOM8N_SUCCESS)) {
				// Update RTC registers.
				RTC_Calibrate(&spsws_ctx.spsws_current_timestamp);
				// Update RTC calibration flags.
				spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_RTC_FIRST_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX);
			}
			// Reload watchdog.
			IWDG_Reload();
			// Compute next state.
			if (spsws_ctx.spsws_por_flag == 0) {
				// Enter standby mode.
				spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			}
			else {
				// NVM POR update.
				spsws_ctx.spsws_state = SPSWS_STATE_NVM_POR_UPDATE;
			}
			break;

		/* NVM POR UPDATE */
		case SPSWS_STATE_NVM_POR_UPDATE:
			// Check RTC calibration;
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_FIRST_CALIBRATION_BIT_IDX)) != 0) {
				// Check if day has changed.
				SPSWS_UpdateTimestampFlags();
				if (spsws_ctx.spsws_day_changed_flag != 0) {
					// Reset daily downlink flag.
					spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
				}
			}
			else {
				// Reset daily downlink flag.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
			}
			// Update previous wake-up timestamp.
			SPSWS_UpdatePwut();
			// Reload watchdog.
			IWDG_Reload();
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* SLEEP */
		case SPSWS_STATE_SLEEP:
			// Write status byte in NVM.
			NVM_WriteByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, spsws_ctx.spsws_status_byte);
			NVM_Disable();
			// Reload watchdog.
			IWDG_Reload();
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23); // RMVF='1'.
			// Clear RTC flags.
			RTC_ClearAlarmFlags();
			// Enter standby mode.
			PWR_EnterStandbyMode();
			break;

		/* UNKNOWN STATE */
		default:
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
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

	/* Init memory */
	FLASH_Init();

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();
	SPSWS_BlinkLed(10);

	/* Init clocks */
	RCC_Init();
	// High speed oscillator.
	if (RCC_SwitchToTcxo16MHz() == 0) {
		while (RCC_SwitchToInternal16MHz() == 0);
	}

	/* Init peripherals */
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
	// Hardware AES.
	AES_Init();

	/* Init components */
	SX1232_Init();
	SKY13317_Init();
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
	}

	return 0;
}
#endif
