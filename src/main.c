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
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvic.h"
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

#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS	120
#define SPSWS_LOCAL_UTC_OFFSET					2
#define SPSWS_NUMBER_OF_HOURS_PER_DAY			24
#define SPSWS_AFTERNOON_HOUR_THRESHOLD			12
#define SPSWS_GEOLOC_TIMEOUT_SECONDS			120

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
	SPSWS_STATE_OFF,
	SPSWS_STATE_SLEEP
} SPSWS_State;

typedef enum {
	SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX,
	SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX,
	SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX,
	SPSWS_STATUS_BYTE_FIRST_RTC_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX,
} SPSWS_StatusBitsIndex;

typedef struct {
	// Global.
	SPSWS_State spsws_state;
	// Time management.
	unsigned char spsws_por_flag;
	unsigned char spsws_hour_changed_flag;
	unsigned char spsws_day_changed_flag;
	unsigned char spsws_is_afternoon_flag;
	unsigned int spsws_lsi_frequency_hz;
	// Wake-up management.
	Timestamp spsws_current_timestamp;
	Timestamp spsws_previous_wake_up_timestamp;
	// Monitoring.
	MONITORING_Data spsws_monitoring_data;
	unsigned char spsws_status_byte;
	// Weather data.
	WEATHER_Data spsws_weather_data;
	// Geoloc.
	Position spsws_geoloc_position;
	unsigned char spsws_geoloc_fix_duration_seconds;
	// Sigfox.
	unsigned char spsws_sfx_uplink_data[SFX_UPLINK_DATA_MAX_SIZE_BYTES];
	unsigned char spsws_sfx_downlink_data[SFX_DOWNLINK_DATA_SIZE_BYTES];
} SPSWS_Context;

/*** SPSWS global variables ***/

static SPSWS_Context spsws_ctx;

/*** SPSWS local functions ***/

#ifdef DEBUG
/* MAKE THE LED BLINK.
 * @param number_of_blinks:	Number of blinks.
 * @return:					None.
 */
void SPSWS_BlinkLed(unsigned char number_of_blinks) {
	unsigned char j = 0;
	unsigned int k = 0;
	for (j=0 ; j<number_of_blinks ; j++) {
		GPIO_Write(&GPIO_LED, 1);
		for (k=0 ; k<10000 ; k++);
		GPIO_Write(&GPIO_LED, 0);
		for (k=0 ; k<10000 ; k++);
	}
}
#endif

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
		// Day (and thus hour) has changed.
		spsws_ctx.spsws_day_changed_flag = 1;
		spsws_ctx.spsws_hour_changed_flag = 1;
	}
	if (spsws_ctx.spsws_current_timestamp.hours != spsws_ctx.spsws_previous_wake_up_timestamp.hours) {
		// Hour as changed.
		spsws_ctx.spsws_hour_changed_flag = 1;
	}
	// Check if we are in afternoon (to enable device geolocation).
	signed char local_hour = (spsws_ctx.spsws_current_timestamp.hours + SPSWS_LOCAL_UTC_OFFSET) % SPSWS_NUMBER_OF_HOURS_PER_DAY;
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

/*** SPSWS main function ***/

#if (defined IM_RTC || defined CM_RTC)
/* MAIN FUNCTION FOR IM_RTC MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init memory */
	NVIC_Init();
	FLASH_Init();
	NVM_Enable();

	/* Init GPIOs (required for clock tree configuration) */
	GPIO_Init();

	/* Init context */
	spsws_ctx.spsws_state = SPSWS_STATE_RESET;
	spsws_ctx.spsws_por_flag = 0;
	spsws_ctx.spsws_hour_changed_flag = 0;
	spsws_ctx.spsws_day_changed_flag = 0;
	spsws_ctx.spsws_is_afternoon_flag = 0;
	spsws_ctx.spsws_geoloc_fix_duration_seconds = 0;
	NVM_ReadByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, &spsws_ctx.spsws_status_byte);
#ifdef IM_RTC
	spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX); // IM = 0b0.
#else
	spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX); // CM = 0b1.
#endif

	/* Local variables */
	unsigned char rtc_use_lse = 0;
	unsigned int max11136_result_12bits = 0;
	NEOM8N_ReturnCode neom8n_return_code = NEOM8N_TIMEOUT;
	unsigned int geoloc_fix_start_time_seconds = 0;
	unsigned char geoloc_timeout = 0;
	sfx_rc_t spsws_sigfox_rc = (sfx_rc_t) SPSWS_SIGFOX_RC;
	sfx_error_t sfx_error = SFX_ERR_NONE;

	/* Main loop */
	while (1) {

		/* Perform state machine */
		switch (spsws_ctx.spsws_state) {

		/* RESET */
		case SPSWS_STATE_RESET:
#ifdef DEBUG
			// Turn LED on.
			GPIO_Write(&GPIO_LED, 1);
#endif
			// Check reset reason.
			if (((RCC -> CSR) & (0b1 << 29)) != 0) {
				// IWDG reset: directly enter standby mode.
				spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			}
			else {
				if (((RCC -> CSR) & (0b111 << 26)) != 0) {
					// SW, NRST or POR reset: directly go to INIT state.
					spsws_ctx.spsws_por_flag = 1;
					spsws_ctx.spsws_state = SPSWS_STATE_INIT;
				}
				else {
					// Other resetor RTC wake-up: standard flow.
					spsws_ctx.spsws_state = SPSWS_STATE_HOUR_CHECK;
				}
			}
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23); // RMVF='1'.
			break;

		/* HOUR CHECK */
		case SPSWS_STATE_HOUR_CHECK:
			// Update flags.
			SPSWS_UpdateTimestampFlags();
			// Check flag.
			if (spsws_ctx.spsws_hour_changed_flag == 0) {
				// False detection due to RTC recalibration.
				spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			}
			else {
				// Valid wake-up.
				spsws_ctx.spsws_state = SPSWS_STATE_NVM_WAKE_UP_UPDATE;
			}
			break;

		/* NVM WAKE UP UPDATE */
		case SPSWS_STATE_NVM_WAKE_UP_UPDATE:
			// Update previous wake-up timestamp.
			SPSWS_UpdatePwut();
			// Check if day changed.
			if (spsws_ctx.spsws_day_changed_flag != 0) {
				// Reset daily flags.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX);
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
				// Reset RTC before starting oscillators.
				RTC_Reset();
				// LSI.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableLsi() << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				// LSE.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableLse() << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			}
			// Watchdog.
			//IWDG_Init();
			//IWDG_Reload();
			// High speed oscillator.
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			spsws_ctx.spsws_status_byte |= (RCC_SwitchToHse() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
				while (RCC_SwitchToHsi() == 0);
			}
			// Get LSI effective frequency (must be called after HSx initialization and before RTC inititialization).
			spsws_ctx.spsws_lsi_frequency_hz = RCC_GetLsiFrequency();
			// Timers (must be called before RTC inititialization to have timeout available).
			TIM21_Init();
			TIM22_Init();
			TIM21_Start();
			TIM22_Start();
			LPTIM1_Init(0);
			// RTC (only at POR).
			if (spsws_ctx.spsws_por_flag != 0) {
				rtc_use_lse = spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				RTC_Init(&rtc_use_lse, spsws_ctx.spsws_lsi_frequency_hz);
				// Update LSE status if RTC failed to start on it.
				if (rtc_use_lse == 0) {
					spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				}
			}
#ifdef CM_RTC
			// External interrupts.
			EXTI_Init();
#endif
			// Analog.
			ADC1_Init();
			// Communication interfaces.
			LPUART1_Init();
			I2C1_Init();
			SPI1_Init();
#ifdef HW2_0
			SPI2_Init();
#endif
			// DMA.
			DMA1_Init();
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
#ifdef CM_RTC
			WIND_Init();
#endif
			// Compute next state.
			if (spsws_ctx.spsws_por_flag == 0) {
				spsws_ctx.spsws_state = SPSWS_STATE_MEASURE;
			}
			else {
				spsws_ctx.spsws_state = SPSWS_STATE_POR;
			}
			break;

		/* STATIC MEASURE */
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
			// Convert channel results to mV.
			MAX11136_GetChannel(MAX11136_CHANNEL_SOLAR_CELL, &max11136_result_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_solar_cell_voltage_mv = (spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv * max11136_result_12bits * 269) / (MAX11136_FULL_SCALE * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_SUPERCAP, &max11136_result_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_supercap_voltage_mv = (spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv * max11136_result_12bits * 269) / (MAX11136_FULL_SCALE * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_LDR, &max11136_result_12bits);
			spsws_ctx.spsws_weather_data.weather_data_light_percent = (max11136_result_12bits / MAX11136_FULL_SCALE) * 100;
			// Retrieve weather sensors data.
			I2C1_PowerOn();
			// Internal temperature/humidity sensor.
			SHT3X_PerformMeasurements(SHT3X_INTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperature(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_temperature_degrees);
			SHT3X_GetHumidity(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_humidity_percent);
			// External temperature/humidity sensor.
			SHT3X_PerformMeasurements(SHT3X_EXTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperature(&spsws_ctx.spsws_weather_data.weather_data_temperature_degrees);
			SHT3X_GetHumidity(&spsws_ctx.spsws_weather_data.weather_data_humidity_percent);
			// External pressure/temperature sensor.
			DPS310_PerformMeasurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_GetPressure(&spsws_ctx.spsws_weather_data.weather_data_pressure_pa);
			// External UV index sensor.
			SI1133_PerformMeasurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_GetUvIndex(&spsws_ctx.spsws_weather_data.weather_data_uv_index);
			// Turn sensors off.
			I2C1_PowerOff();
#ifdef CM_RTC
			// Retrieve wind measurements.
			WIND_GetSpeed(&spsws_ctx.spsws_weather_data.weather_data_average_wind_speed_mh, &spsws_ctx.spsws_weather_data.weather_data_peak_wind_speed_mh);
			WIND_GetDirection(&spsws_ctx.spsws_weather_data.weather_data_average_wind_direction_degrees);
			// Retrieve rain measurements.
			//RAIN_GetPluviometry(&spsws_ctx.spsws_weather_data.weather_data_rain_mm);
#endif
			// Read status byte.
			spsws_ctx.spsws_monitoring_data.monitoring_data_status_byte = spsws_ctx.spsws_status_byte;
			// Compute next state.
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
			SIGFOX_API_close();
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_WEATHER_DATA;
			break;

		/* WEATHER DATA */
		case SPSWS_STATE_WEATHER_DATA:
			// Build Sigfox frame.
			WEATHER_BuildSigfoxData(&spsws_ctx.spsws_weather_data, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sfx_uplink_data, WEATHER_SIGFOX_DATA_LENGTH, spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX)) == 0) {
				// Perform RTC calibration.
				spsws_ctx.spsws_state = SPSWS_STATE_RTC_CALIBRATION;
			}
			else {
				if (((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX)) == 0) && (spsws_ctx.spsws_is_afternoon_flag != 0)) {
					// Perform device geolocation.
					spsws_ctx.spsws_state = SPSWS_STATE_GEOLOC;
				}
				else {
					// Enter standby mode.
					spsws_ctx.spsws_state = SPSWS_STATE_OFF;
				}
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
			// Reset all daily flags.
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_FIRST_RTC_CALIBRATION_BIT_IDX);
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX);
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX);
			// Perform first RTC calibration.
			spsws_ctx.spsws_state = SPSWS_STATE_RTC_CALIBRATION;
			break;

		/* GEOLOC */
		case SPSWS_STATE_GEOLOC:
			// Get position from GPS.
			LPUART1_PowerOn();
			geoloc_fix_start_time_seconds = TIM22_GetSeconds();
			neom8n_return_code = NEOM8N_GetPosition(&spsws_ctx.spsws_geoloc_position, SPSWS_GEOLOC_TIMEOUT_SECONDS);
			LPUART1_PowerOff();
			// Update flag whatever the result.
			spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX);
			// Parse result.
			if (neom8n_return_code == NEOM8N_SUCCESS) {
				// Get fix duration and update flag.
				spsws_ctx.spsws_geoloc_fix_duration_seconds = TIM22_GetSeconds() - geoloc_fix_start_time_seconds;
			}
			else {
				// Set fix duration to timeout.
				spsws_ctx.spsws_geoloc_fix_duration_seconds = SPSWS_GEOLOC_TIMEOUT_SECONDS;
				geoloc_timeout = 1;
			}
			// Build Sigfox frame.
			GEOLOC_BuildSigfoxData(&spsws_ctx.spsws_geoloc_position, spsws_ctx.spsws_geoloc_fix_duration_seconds, geoloc_timeout, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&spsws_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sfx_uplink_data, (geoloc_timeout ? GEOLOC_TIMEOUT_SIGFOX_DATA_LENGTH : GEOLOC_SIGFOX_DATA_LENGTH), spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			break;

		/* RTC CALIBRATION */
		case SPSWS_STATE_RTC_CALIBRATION:
			// Get current timestamp from GPS.{
			LPUART1_PowerOn();
			neom8n_return_code = NEOM8N_GetTimestamp(&spsws_ctx.spsws_current_timestamp, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS);
			LPUART1_PowerOff();
			// Calibrate RTC if timestamp is available.
			if (neom8n_return_code == NEOM8N_SUCCESS) {
				// Update RTC registers.
				RTC_Calibrate(&spsws_ctx.spsws_current_timestamp);
				// Update PWUT when first calibration.
				if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_FIRST_RTC_CALIBRATION_BIT_IDX)) == 0) {
					SPSWS_UpdatePwut();
				}
				// Update calibration flags.
				spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_FIRST_RTC_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX);
			}
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			break;

		/* OFF */
		case SPSWS_STATE_OFF:
			// Clear POR flag.
			spsws_ctx.spsws_por_flag = 0;
			// Switch to internal clock.
			RCC_SwitchToHsi();
			// Turn peripherals off.
			ADC1_Disable();
#ifdef HW2_0
			SX1232_Tcxo(0);
#endif
#ifdef IM_RTC
			TIM21_Disable();
			TIM22_Disable();
			LPTIM1_Disable();
#ifdef HW2_0
			SPI2_Disable();
#endif
#endif
#ifdef HW2_0
			SPI1_Disable();
#endif
			DMA1_Disable();
			LPUART1_Disable();
			I2C1_Disable();
			AES_Disable();
			// Store status byte in NVM.
			NVM_WriteByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, spsws_ctx.spsws_status_byte);
			NVM_Disable();
#ifdef DEBUG
			GPIO_Write(&GPIO_LED, 0);
#endif
#ifdef CM_RTC
			// Re-start continuous measurements.
			WIND_ResetData();
			WIND_StartContinuousMeasure();
#endif
			// Clear RTC flags.
			RTC_ClearAlarmFlags();
#ifdef CM_RTC
			NVIC_EnableInterrupt(IT_RTC);
#endif
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;

		/* SLEEP */
		case SPSWS_STATE_SLEEP:
#ifdef IM_RTC
			// Enter standby mode.
			PWR_EnterStandbyMode();
#endif
#ifdef CM_RTC
			// Enter sleep mode.
			PWR_EnterSleepMode();
			// Check RTC flag.
			if (RTC_GetAlarmFlag() != 0) {
				// Stop continuous measurements.
				WIND_StopContinuousMeasure();
				// Wake-up.
				spsws_ctx.spsws_state = SPSWS_STATE_RESET;
			}
#endif
			break;

		/* UNKNOWN STATE */
		default:
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_OFF;
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

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init memory */
	NVIC_Init();
	FLASH_Init();
	NVM_Enable();

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();
#ifdef DEBUG
	SPSWS_BlinkLed(10);
#endif

	/* Init clocks */
	RCC_Init();
	// High speed oscillator.
	if (RCC_SwitchToHse() == 0) {
		while (RCC_SwitchToHsi() == 0);
	}

	/* Init peripherals */
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
	// External interrupts.
	EXTI_Init();
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
#ifdef HW2_0
	SX1232_Tcxo(1);
#endif
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
		// Perform AT commands parsing.
		AT_Task();
	}

	return 0;
}
#endif
