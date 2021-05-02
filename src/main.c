/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Registers
#include "rcc_reg.h"
// Peripherals.
#include "adc.h"
#include "aes.h"
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
#include "mode.h"
#include "monitoring.h"
#include "rain.h"
#include "sigfox_api.h"
#include "weather.h"

/*** SPSWS macros ***/

// Time management.
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS		180
#define SPSWS_LOCAL_UTC_OFFSET_WINTER				1
#define SPSWS_LOCAL_UTC_OFFSET_SUMMER				2
#define SPSWS_WINTER_TIME_LAST_MONTH				3
#define SPSWS_WINTER_TIME_FIRST_MONTH				11
#define SPSWS_NUMBER_OF_HOURS_PER_DAY				24
#define SPSWS_AFTERNOON_HOUR_THRESHOLD				12
// Geoloc.
#define SPSWS_GEOLOC_TIMEOUT_SECONDS				120
// Sigfox.
#define SPSWS_SIGFOX_UPLINK_DATA_MAX_LENGTH_BYTES	12
#define SPSWS_SIGFOX_DOWNLINK_DATA_SIZE_BYTES		8
#define SPSWS_SIGFOX_RC_STD_CONFIG_SIZE				3

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
	unsigned int spsws_geoloc_fix_duration_seconds;
	unsigned char spsws_geoloc_timeout_flag;
	// Sigfox.
	sfx_rc_t spsws_sfx_rc;
	sfx_u32 spsws_sfx_rc_std_config[SPSWS_SIGFOX_RC_STD_CONFIG_SIZE];
	unsigned char spsws_sfx_uplink_data[SPSWS_SIGFOX_UPLINK_DATA_MAX_LENGTH_BYTES];
	unsigned char spsws_sfx_downlink_data[SPSWS_SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
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
		for (k=0 ; k<20000 ; k++) {
			// Poll a bit always read as '0'.
			// This is required to avoid for loop removal by compiler (size optimization for HW1.0).
			if (((RCC -> CR) & (0b1 << 24)) != 0) {
				break;
			}
		}
		GPIO_Write(&GPIO_LED, 0);
		for (k=0 ; k<20000 ; k++) {
			// Poll a bit always read as '0'.
			// This is required to avoid for loop removal by compiler (size optimization for HW1.0).
			if (((RCC -> CR) & (0b1 << 24)) != 0) {
				break;
			}
		}
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
	NVM_Enable();
	NVM_ReadByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), &nvm_byte);
	spsws_ctx.spsws_previous_wake_up_timestamp.year = (nvm_byte << 8);
	NVM_ReadByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), &nvm_byte);
	spsws_ctx.spsws_previous_wake_up_timestamp.year |= nvm_byte;
	NVM_ReadByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.month);
	NVM_ReadByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.date);
	NVM_ReadByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, &spsws_ctx.spsws_previous_wake_up_timestamp.hours);
	NVM_Disable();
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
	unsigned char local_utc_offset = SPSWS_LOCAL_UTC_OFFSET_WINTER;
	if ((spsws_ctx.spsws_current_timestamp.month > SPSWS_WINTER_TIME_LAST_MONTH) && (spsws_ctx.spsws_current_timestamp.month < SPSWS_WINTER_TIME_FIRST_MONTH)) {
		local_utc_offset = SPSWS_LOCAL_UTC_OFFSET_SUMMER;
	}
	signed char local_hour = (spsws_ctx.spsws_current_timestamp.hours + local_utc_offset) % SPSWS_NUMBER_OF_HOURS_PER_DAY;
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
	NVM_Enable();
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), ((spsws_ctx.spsws_current_timestamp.year & 0xFF00) >> 8));
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), ((spsws_ctx.spsws_current_timestamp.year & 0x00FF) >> 0));
	NVM_WriteByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.month);
	NVM_WriteByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.date);
	NVM_WriteByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, spsws_ctx.spsws_current_timestamp.hours);
	NVM_Disable();
}

/*** SPSWS main function ***/

#if (defined IM || defined CM)
/* MAIN FUNCTION FOR IM MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_Init();
	// Init GPIOs.
	GPIO_Init(); // Required for clock tree configuration.
	EXTI_Init(); // Required to clear RTC flags (EXTI 17).
	// Init clock and power modules.
	RCC_Init();
	PWR_Init();
	// Init context.
	spsws_ctx.spsws_state = SPSWS_STATE_RESET;
	spsws_ctx.spsws_por_flag = 0;
	spsws_ctx.spsws_hour_changed_flag = 0;
	spsws_ctx.spsws_day_changed_flag = 0;
	spsws_ctx.spsws_is_afternoon_flag = 0;
	spsws_ctx.spsws_geoloc_timeout_flag = 0;
	spsws_ctx.spsws_geoloc_fix_duration_seconds = 0;
	NVM_Enable();
	NVM_ReadByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, &spsws_ctx.spsws_status_byte);
	NVM_Disable();
#ifdef IM
	spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX); // IM = 0b0.
#else
	spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX); // CM = 0b1.
#endif
	unsigned char idx = 0;
	spsws_ctx.spsws_sfx_rc = (sfx_rc_t) RC1;
	for (idx=0 ; idx<SPSWS_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) spsws_ctx.spsws_sfx_rc_std_config[idx] = 0;
	// Local variables.
	unsigned char rtc_use_lse = 0;
	unsigned int max11136_bandgap_12bits = 0;
	unsigned int max11136_channel_12bits = 0;
	NEOM8N_ReturnCode neom8n_return_code = NEOM8N_TIMEOUT;
	sfx_error_t sfx_error = SFX_ERR_NONE;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.spsws_state) {
		// RESET.
		case SPSWS_STATE_RESET:
			IWDG_Reload();
#ifdef DEBUG
			// Turn LED on.
			GPIO_Write(&GPIO_LED, 1);
#endif
			// Check reset reason.
			if (((RCC -> CSR) & (0b1111 << 26)) != 0) {
				// IWDG, SW, NRST or POR reset: directly go to INIT state.
				spsws_ctx.spsws_por_flag = 1;
				spsws_ctx.spsws_state = SPSWS_STATE_INIT;
			}
			else {
				// Other reset or RTC wake-up: standard flow.
				spsws_ctx.spsws_state = SPSWS_STATE_HOUR_CHECK;
			}
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23); // RMVF='1'.
			break;
		// HOUR CHECK.
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
				spsws_ctx.spsws_hour_changed_flag = 0; // Reset flag.
			}
			break;
		// NVM WAKE UP UPDATE.
		case SPSWS_STATE_NVM_WAKE_UP_UPDATE:
			// Update previous wake-up timestamp.
			SPSWS_UpdatePwut();
			// Check if day changed.
			if (spsws_ctx.spsws_day_changed_flag != 0) {
				// Reset daily flags.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_RTC_CALIBRATION_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX);
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX);
				// Reset flags.
				spsws_ctx.spsws_day_changed_flag = 0;
				spsws_ctx.spsws_hour_changed_flag = 0;
				spsws_ctx.spsws_is_afternoon_flag = 0;
			}
			// Go to INIT state.
			spsws_ctx.spsws_state = SPSWS_STATE_INIT;
			break;
		// INIT.
		case SPSWS_STATE_INIT:
			// Low speed oscillators and watchdog (only at POR).
			if (spsws_ctx.spsws_por_flag != 0) {
				// Start independant watchdog.
				IWDG_Init();
				// Reset RTC before starting oscillators.
				RTC_Reset();
				// LSI.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableLsi() << SPSWS_STATUS_BYTE_LSI_STATUS_BIT_IDX);
				// LSE.
				spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				spsws_ctx.spsws_status_byte |= (RCC_EnableLse() << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			}
			// High speed oscillator.
			IWDG_Reload();
			RCC_EnableGpio();
			spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			spsws_ctx.spsws_status_byte |= (RCC_SwitchToHse() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
				RCC_SwitchToHsi();
			}
			// Get LSI effective frequency (must be called after HSx initialization and before RTC inititialization).
			RCC_GetLsiFrequency(&spsws_ctx.spsws_lsi_frequency_hz);
			IWDG_Reload();
			// Timers.
			LPTIM1_Init();
			// RTC (only at POR).
			if (spsws_ctx.spsws_por_flag != 0) {
				rtc_use_lse = spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				RTC_Init(&rtc_use_lse, spsws_ctx.spsws_lsi_frequency_hz);
				// Update LSE status if RTC failed to start on it.
				if (rtc_use_lse == 0) {
					spsws_ctx.spsws_status_byte &= ~(0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				}
			}
			IWDG_Reload();
			// Communication interfaces.
#ifdef HW1_0
			USART2_Init();
#endif
#ifdef HW2_0
			USART1_Init();
#endif
			LPUART1_Init(rtc_use_lse);
			I2C1_Init();
			SPI1_Init();
#ifdef HW2_0
			SPI2_Init();
#endif
			// Init components.
			SX1232_Init();
			SX1232_Tcxo(1);
			SKY13317_Init();
			NEOM8N_Init();
			MAX11136_Init();
			SHT3X_Init();
			DPS310_Init();
			SI1133_Init();
#ifdef CM
			WIND_Init();
			RAIN_Init();
#endif
			// Compute next state.
			if (spsws_ctx.spsws_por_flag == 0) {
				spsws_ctx.spsws_state = SPSWS_STATE_MEASURE;
			}
			else {
				spsws_ctx.spsws_state = SPSWS_STATE_POR;
			}
			break;
		// STATIC MEASURE.
		case SPSWS_STATE_MEASURE:
			IWDG_Reload();
			// Retrieve internal ADC data.
			ADC1_PerformAllMeasurements();
			ADC1_GetMcuTemperatureComp2(&spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_temperature_degrees);
			ADC1_GetMcuVoltage(&spsws_ctx.spsws_monitoring_data.monitoring_data_mcu_voltage_mv);
			// Retrieve external ADC data.
#ifdef HW1_0
			SPI1_PowerOn();
#endif
#ifdef HW2_0
			I2C1_PowerOn(); // Must be called before ADC since LDR is on the MSM module (powered by I2C sensors supply).
			SPI2_PowerOn();
#endif
			IWDG_Reload();
			MAX11136_PerformMeasurements();

#ifdef HW1_0
			SPI1_PowerOff();
#endif
#ifdef HW2_0
			SPI2_PowerOff();
#endif
			// Convert channel results to mV.
			MAX11136_GetChannel(MAX11136_CHANNEL_BANDGAP, &max11136_bandgap_12bits);
			MAX11136_GetChannel(MAX11136_CHANNEL_SOLAR_CELL, &max11136_channel_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_solar_cell_voltage_mv = (max11136_channel_12bits * MAX11136_BANDGAP_VOLTAGE_MV * 269) / (max11136_bandgap_12bits * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_SUPERCAP, &max11136_channel_12bits);
			spsws_ctx.spsws_monitoring_data.monitoring_data_supercap_voltage_mv = (max11136_channel_12bits * MAX11136_BANDGAP_VOLTAGE_MV * 269) / (max11136_bandgap_12bits * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_LDR, &max11136_channel_12bits);
			spsws_ctx.spsws_weather_data.weather_data_light_percent = (max11136_channel_12bits * 100) / MAX11136_FULL_SCALE;
			// Retrieve weather sensors data.
#ifdef HW1_0
			I2C1_PowerOn();
#endif
			// Internal temperature/humidity sensor.
			IWDG_Reload();
			SHT3X_PerformMeasurements(SHT3X_INTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperature(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_temperature_degrees);
			SHT3X_GetHumidity(&spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_humidity_percent);
			// External temperature/humidity sensor.
#ifdef HW1_0
			spsws_ctx.spsws_weather_data.weather_data_temperature_degrees = spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_temperature_degrees;
			spsws_ctx.spsws_weather_data.weather_data_humidity_percent = spsws_ctx.spsws_monitoring_data.monitoring_data_pcb_humidity_percent;
#endif
#ifdef HW2_0
			IWDG_Reload();
			SHT3X_PerformMeasurements(SHT3X_EXTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperature(&spsws_ctx.spsws_weather_data.weather_data_temperature_degrees);
			SHT3X_GetHumidity(&spsws_ctx.spsws_weather_data.weather_data_humidity_percent);
#endif
			// External pressure/temperature sensor.
			IWDG_Reload();
			DPS310_PerformMeasurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_GetPressure(&spsws_ctx.spsws_weather_data.weather_data_pressure_pa);
			// External UV index sensor.
			IWDG_Reload();
			SI1133_PerformMeasurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_GetUvIndex(&spsws_ctx.spsws_weather_data.weather_data_uv_index);
			// Turn sensors off.
			I2C1_PowerOff();
#ifdef CM
			IWDG_Reload();
			// Retrieve wind measurements.
			WIND_GetSpeed(&spsws_ctx.spsws_weather_data.weather_data_average_wind_speed_mh, &spsws_ctx.spsws_weather_data.weather_data_peak_wind_speed_mh);
			WIND_GetDirection(&spsws_ctx.spsws_weather_data.weather_data_average_wind_direction_degrees);
			// Retrieve rain measurements.
			RAIN_GetPluviometry(&spsws_ctx.spsws_weather_data.weather_data_rain_mm);
#endif
			// Read status byte.
			spsws_ctx.spsws_monitoring_data.monitoring_data_status_byte = spsws_ctx.spsws_status_byte;
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_MONITORING;
			break;
		// MONITORING.
		case SPSWS_STATE_MONITORING:
			IWDG_Reload();
			// Build Sigfox frame.
			MONITORING_BuildSigfoxData(&spsws_ctx.spsws_monitoring_data, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sfx_uplink_data, MONITORING_SIGFOX_DATA_LENGTH, spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_WEATHER_DATA;
			break;
		// WEATHER DATA.
		case SPSWS_STATE_WEATHER_DATA:
			IWDG_Reload();
			// Build Sigfox frame.
			WEATHER_BuildSigfoxData(&spsws_ctx.spsws_weather_data, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
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
		// POR.
		case SPSWS_STATE_POR:
			IWDG_Reload();
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
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
		// GEOLOC.
		case SPSWS_STATE_GEOLOC:
			IWDG_Reload();
			// Get position from GPS.
			LPUART1_PowerOn();
			neom8n_return_code = NEOM8N_GetPosition(&spsws_ctx.spsws_geoloc_position, SPSWS_GEOLOC_TIMEOUT_SECONDS, 0, &spsws_ctx.spsws_geoloc_fix_duration_seconds);
			LPUART1_PowerOff();
			// Update flag whatever the result.
			spsws_ctx.spsws_status_byte |= (0b1 << SPSWS_STATUS_BYTE_DAILY_GEOLOC_BIT_IDX);
			// Parse result.
			if (neom8n_return_code != NEOM8N_SUCCESS) {
				spsws_ctx.spsws_geoloc_timeout_flag = 1;
			}
			IWDG_Reload();
			// Build Sigfox frame.
			GEOLOC_BuildSigfoxData(&spsws_ctx.spsws_geoloc_position, spsws_ctx.spsws_geoloc_fix_duration_seconds, spsws_ctx.spsws_geoloc_timeout_flag, spsws_ctx.spsws_sfx_uplink_data);
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sfx_uplink_data, (spsws_ctx.spsws_geoloc_timeout_flag ? GEOLOC_TIMEOUT_SIGFOX_DATA_LENGTH : GEOLOC_SIGFOX_DATA_LENGTH), spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset geoloc variables.
			spsws_ctx.spsws_geoloc_timeout_flag = 0;
			spsws_ctx.spsws_geoloc_fix_duration_seconds = 0;
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			break;
		// RTC CALIBRATION.
		case SPSWS_STATE_RTC_CALIBRATION:
			IWDG_Reload();
			// Turn radio TCXO off since Sigfox is not required anymore.
			SX1232_Tcxo(0);
			// Get current timestamp from GPS.{
			LPUART1_PowerOn();
			neom8n_return_code = NEOM8N_GetTimestamp(&spsws_ctx.spsws_current_timestamp, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS, 0);
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
		// OFF.
		case SPSWS_STATE_OFF:
			IWDG_Reload();
			// Clear POR flag.
			spsws_ctx.spsws_por_flag = 0;
			// Turn peripherals off.
			SX1232_DisableGpio();
			SKY13317_DisableGpio();
			MAX11136_DisableGpio();
			SX1232_Tcxo(0);
#ifdef HW2_0
			SPI2_Disable();
#endif
			TIM2_Disable();
			LPTIM1_Disable();
			SPI1_Disable();
			LPUART1_Disable();
			I2C1_Disable();
			AES_Disable();
			// Store status byte in NVM.
			NVM_Enable();
			NVM_WriteByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, spsws_ctx.spsws_status_byte);
			NVM_Disable();
			// Switch to internal MSI 65kHz (must be called before WIND functions to init LPTIM with right clock frequency).
			RCC_SwitchToMsi();
			RCC_DisableGpio();
#ifdef CM
			// Re-start continuous measurements.
			WIND_ResetData();
			RAIN_ResetData();
			WIND_StartContinuousMeasure();
			RAIN_StartContinuousMeasure();
#endif
			// Clear RTC flags.
			RTC_ClearAlarmAFlag();
			RTC_ClearAlarmBFlag();
			NVIC_EnableInterrupt(NVIC_IT_RTC);
#ifdef DEBUG
			// Turn LED off.
			GPIO_Write(&GPIO_LED, 0);
#endif
			// Enter sleep mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;
		// SLEEP.
		case SPSWS_STATE_SLEEP:
			IWDG_Reload();
			// Enter sleep mode.
			PWR_EnterLowPowerSleepMode();
			// Check RTC flags.
			if (RTC_GetAlarmBFlag() != 0) {
#ifdef CM
				// Call WIND callback.
				WIND_MeasurementPeriodCallback();
#endif
				// Clear RTC flags.
				RTC_ClearAlarmBFlag();
			}
			if (RTC_GetAlarmAFlag() != 0) {
				// Disable RTC interrupt.
				NVIC_DisableInterrupt(NVIC_IT_RTC);
#ifdef CM
				// Stop continuous measurements.
				WIND_StopContinuousMeasure();
				RAIN_StopContinuousMeasure();
#endif
				// Clear RTC flags.
				RTC_ClearAlarmAFlag();
				// Wake-up.
				spsws_ctx.spsws_state = SPSWS_STATE_RESET;
			}
			break;
		// UNKNOWN STATE.
		default:
			IWDG_Reload();
			// Enter standby mode.
			spsws_ctx.spsws_state = SPSWS_STATE_OFF;
			break;
		}
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
	// Init memory.
	NVIC_Init();
	FLASH_SetLatency(1);
	NVM_Enable();
	// Init GPIO (required for clock tree configuration).
	GPIO_Init();
#ifdef DEBUG
	SPSWS_BlinkLed(10);
#endif
	// Init clocks.
	RCC_Init();
	RCC_EnableGpio();
	RTC_Reset();
	RCC_EnableLsi();
	// High speed oscillator.
	if (RCC_SwitchToHse() == 0) {
		RCC_SwitchToHsi();
	}
	// Init peripherals.
	unsigned int lsi_frequency_hz = RCC_GetLsiFrequency();
	// Timers.
	TIM21_Init();
	TIM22_Init();
	TIM21_Start();
	TIM22_Start();
	LPTIM1_Init(LPTIM_MODE_DELAY);
	// RTC.
	RTC_Init(0, lsi_frequency_hz);
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
	// Init components.
	SX1232_Init();
	SX1232_Tcxo(1);
	SKY13317_Init();
#ifdef AT_COMMANDS_GPS
	NEOM8N_Init();
#endif
#ifdef AT_COMMANDS_SENSORS
	MAX11136_Init();
	WIND_Init();
	RAIN_Init();
	SHT3X_Init();
	DPS310_Init();
	SI1133_Init();
#endif
	// Init applicative layers.
	AT_Init();
	// Main loop.
	while (1) {
		// Perform AT commands parsing.
		AT_Task();
		// Check RTC flag for wind measurements.
		if (RTC_GetAlarmBFlag() != 0) {
			// Call WIND callback.
			WIND_MeasurementPeriodCallback();
			RTC_ClearAlarmBFlag();
		}
	}
	return 0;
}
#endif
