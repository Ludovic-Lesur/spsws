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
#include "mode.h"
#include "rain.h"
#include "sigfox_api.h"

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
#ifdef IM
#define SPSWS_SIGFOX_WEATHER_DATA_LENGTH			6
#else
#define SPSWS_SIGFOX_WEATHER_DATA_LENGTH			10
#endif
#define SPSWS_SIGFOX_MONITORING_DATA_LENGTH			9
#define SPSWS_SIGFOX_GEOLOC_DATA_LENGTH				11
#define SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH		1

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

// Sigfox weather frame data format.
typedef union {
	unsigned char raw_frame[SPSWS_SIGFOX_WEATHER_DATA_LENGTH];
	struct {
		unsigned temperature_degrees : 8;
		unsigned humidity_percent : 8;
		unsigned light_percent : 8;
		unsigned uv_index : 8;
		unsigned absolute_pressure_tenth_hpa : 16;
#ifdef CM
		unsigned average_wind_speed_kmh : 8;
		unsigned peak_wind_speed_kmh : 8;
		unsigned average_wind_direction_two_degrees : 8;
		unsigned rain_mm : 8;
#endif
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_SigfoxWeatherData;

// Sigfox monitoring frame data format.
typedef union {
	unsigned char raw_frame[SPSWS_SIGFOX_MONITORING_DATA_LENGTH];
	struct {
		unsigned mcu_temperature_degrees : 8;
		unsigned pcb_temperature_degrees : 8;
		unsigned pcb_humidity_percent : 8;
		unsigned solar_cell_voltage_mv : 16;
		unsigned supercap_voltage_mv : 12;
		unsigned mcu_voltage_mv : 12;
		unsigned status_byte : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_SigfoxMonitoringData;

// Sigfox geolocation frame data format.
typedef union {
	unsigned char raw_frame[SPSWS_SIGFOX_GEOLOC_DATA_LENGTH];
	struct {
		unsigned latitude_degrees : 8;
		unsigned latitude_minutes : 6;
		unsigned latitude_seconds : 17;
		unsigned latitude_north_flag : 1;
		unsigned longitude_degrees : 8;
		unsigned longitude_minutes : 6;
		unsigned longitude_seconds : 17;
		unsigned longitude_east_flag : 1;
		unsigned altitude_meters : 16;
		unsigned gps_fix_duration : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_SigfoxGeolocData;

typedef struct {
	// Global.
	SPSWS_State spsws_state;
	// Clocks.
	unsigned int spsws_lsi_frequency_hz;
	unsigned char spsws_lse_running;
	// Time management.
	unsigned char spsws_por_flag;
	unsigned char spsws_hour_changed_flag;
	unsigned char spsws_day_changed_flag;
	unsigned char spsws_is_afternoon_flag;
	// Wake-up management.
	Timestamp spsws_current_timestamp;
	Timestamp spsws_previous_wake_up_timestamp;
	// Monitoring.
	unsigned char spsws_status_byte;
	SPSWS_SigfoxMonitoringData spsws_sigfox_monitoring_data;
	// Weather data.
	SPSWS_SigfoxWeatherData spsws_sigfox_weather_data;
	// Geoloc.
	Position spsws_geoloc_position;
	unsigned int spsws_geoloc_fix_duration_seconds;
	unsigned char spsws_geoloc_timeout_flag;
	SPSWS_SigfoxGeolocData spsws_sigfox_geoloc_data;
	// Sigfox.
	sfx_rc_t spsws_sfx_rc;
	sfx_u32 spsws_sfx_rc_std_config[SPSWS_SIGFOX_RC_STD_CONFIG_SIZE];
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
	spsws_ctx.spsws_lsi_frequency_hz = 0;
	spsws_ctx.spsws_lse_running = 0;
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
	unsigned int max11136_bandgap_12bits = 0;
	unsigned int max11136_channel_12bits = 0;
	unsigned char generic_data_u8 = 0;
	unsigned int generic_data_u32_1 = 0;
	unsigned int generic_data_u32_2 = 0;
	NEOM8N_ReturnCode neom8n_return_code = NEOM8N_TIMEOUT;
	sfx_error_t sfx_error = SFX_ERR_NONE;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.spsws_state) {
		// RESET.
		case SPSWS_STATE_RESET:
			IWDG_Reload();
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
				spsws_ctx.spsws_lse_running = spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				RTC_Init(&spsws_ctx.spsws_lse_running, spsws_ctx.spsws_lsi_frequency_hz);
				// Update LSE status if RTC failed to start on it.
				if (spsws_ctx.spsws_lse_running == 0) {
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
			LPUART1_Init(spsws_ctx.spsws_lse_running);
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
			ADC1_Init();
			ADC1_PerformAllMeasurements();
			ADC1_Disable();
			ADC1_GetMcuTemperatureComp1(&generic_data_u8);
			spsws_ctx.spsws_sigfox_monitoring_data.field.mcu_temperature_degrees = generic_data_u8;
			ADC1_GetMcuVoltage(&generic_data_u32_1);
			spsws_ctx.spsws_sigfox_monitoring_data.field.mcu_voltage_mv = generic_data_u32_1;
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
			spsws_ctx.spsws_sigfox_monitoring_data.field.solar_cell_voltage_mv = (max11136_channel_12bits * MAX11136_BANDGAP_VOLTAGE_MV * 269) / (max11136_bandgap_12bits * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_SUPERCAP, &max11136_channel_12bits);
			spsws_ctx.spsws_sigfox_monitoring_data.field.supercap_voltage_mv = (max11136_channel_12bits * MAX11136_BANDGAP_VOLTAGE_MV * 269) / (max11136_bandgap_12bits * 34);
			MAX11136_GetChannel(MAX11136_CHANNEL_LDR, &max11136_channel_12bits);
			spsws_ctx.spsws_sigfox_weather_data.field.light_percent = (max11136_channel_12bits * 100) / MAX11136_FULL_SCALE;
			// Retrieve weather sensors data.
#ifdef HW1_0
			I2C1_PowerOn();
#endif
			// Internal temperature/humidity sensor.
			IWDG_Reload();
			SHT3X_PerformMeasurements(SHT3X_INTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperatureComp1(&generic_data_u8);
			spsws_ctx.spsws_sigfox_monitoring_data.field.pcb_temperature_degrees = generic_data_u8;
			SHT3X_GetHumidity(&generic_data_u8);
			spsws_ctx.spsws_sigfox_monitoring_data.field.pcb_humidity_percent = generic_data_u8;
			// External temperature/humidity sensor.
#ifdef HW1_0
			spsws_ctx.spsws_sigfox_weather_data.field.temperature_degrees = spsws_ctx.spsws_sigfox_monitoring_data.field.pcb_temperature_degrees;
			spsws_ctx.spsws_sigfox_weather_data.field.humidity_percent = spsws_ctx.spsws_sigfox_monitoring_data.field.pcb_humidity_percent;
#endif
#ifdef HW2_0
			IWDG_Reload();
			SHT3X_PerformMeasurements(SHT3X_EXTERNAL_I2C_ADDRESS);
			SHT3X_GetTemperatureComp1(&generic_data_u8);
			spsws_ctx.spsws_sigfox_weather_data.field.temperature_degrees = generic_data_u8;
			SHT3X_GetHumidity(&generic_data_u8);
			spsws_ctx.spsws_sigfox_weather_data.field.humidity_percent = generic_data_u8;
#endif
			// External pressure/temperature sensor.
			IWDG_Reload();
			DPS310_PerformMeasurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_GetPressure(&generic_data_u32_1);
			spsws_ctx.spsws_sigfox_weather_data.field.absolute_pressure_tenth_hpa = (generic_data_u32_1 == DPS310_PRESSURE_ERROR_VALUE) ? 0xFFFF : (generic_data_u32_1 / 10);
			// External UV index sensor.
			IWDG_Reload();
			SI1133_PerformMeasurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_GetUvIndex(&generic_data_u8);
			spsws_ctx.spsws_sigfox_weather_data.field.uv_index = generic_data_u8;
			// Turn sensors off.
			I2C1_PowerOff();
#ifdef CM
			IWDG_Reload();
			// Retrieve wind measurements.
			WIND_GetSpeed(&generic_data_u32_1, &generic_data_u32_2);
			spsws_ctx.spsws_sigfox_weather_data.field.average_wind_speed_kmh = (generic_data_u32_1 / 1000);
			spsws_ctx.spsws_sigfox_weather_data.field.peak_wind_speed_kmh = (generic_data_u32_2 / 1000);
			WIND_GetDirection(&generic_data_u32_1);
			spsws_ctx.spsws_sigfox_weather_data.field.average_wind_direction_two_degrees = (generic_data_u32_1 == WIND_DIRECTION_ERROR_VALUE) ? 0xFF : (generic_data_u32_1 / 2);
			// Retrieve rain measurements.
			RAIN_GetPluviometry(&generic_data_u8);
			spsws_ctx.spsws_sigfox_weather_data.field.rain_mm = generic_data_u8;
#endif
			// Read status byte.
			spsws_ctx.spsws_sigfox_monitoring_data.field.status_byte = spsws_ctx.spsws_status_byte;
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_MONITORING;
			break;
		// MONITORING.
		case SPSWS_STATE_MONITORING:
			IWDG_Reload();
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sigfox_monitoring_data.raw_frame, SPSWS_SIGFOX_MONITORING_DATA_LENGTH, spsws_ctx.spsws_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			spsws_ctx.spsws_state = SPSWS_STATE_WEATHER_DATA;
			break;
		// WEATHER DATA.
		case SPSWS_STATE_WEATHER_DATA:
			IWDG_Reload();
			// Send uplink weather frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sigfox_weather_data.raw_frame, SPSWS_SIGFOX_WEATHER_DATA_LENGTH, spsws_ctx.spsws_sfx_downlink_data, 2, 0);
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
			if (neom8n_return_code == NEOM8N_SUCCESS) {
				// Build frame.
				spsws_ctx.spsws_sigfox_geoloc_data.field.latitude_degrees = spsws_ctx.spsws_geoloc_position.lat_degrees;
				spsws_ctx.spsws_sigfox_geoloc_data.field.latitude_minutes = spsws_ctx.spsws_geoloc_position.lat_minutes;
				spsws_ctx.spsws_sigfox_geoloc_data.field.latitude_seconds = spsws_ctx.spsws_geoloc_position.lat_seconds;
				spsws_ctx.spsws_sigfox_geoloc_data.field.latitude_north_flag = spsws_ctx.spsws_geoloc_position.lat_north_flag;
				spsws_ctx.spsws_sigfox_geoloc_data.field.longitude_degrees = spsws_ctx.spsws_geoloc_position.long_degrees;
				spsws_ctx.spsws_sigfox_geoloc_data.field.longitude_minutes = spsws_ctx.spsws_geoloc_position.long_minutes;
				spsws_ctx.spsws_sigfox_geoloc_data.field.longitude_seconds = spsws_ctx.spsws_geoloc_position.long_seconds;
				spsws_ctx.spsws_sigfox_geoloc_data.field.longitude_east_flag = spsws_ctx.spsws_geoloc_position.long_east_flag;
				spsws_ctx.spsws_sigfox_geoloc_data.field.altitude_meters = spsws_ctx.spsws_geoloc_position.altitude;
				spsws_ctx.spsws_sigfox_geoloc_data.field.gps_fix_duration = spsws_ctx.spsws_geoloc_fix_duration_seconds;
			}
			else {
				spsws_ctx.spsws_sigfox_geoloc_data.raw_frame[0] = spsws_ctx.spsws_geoloc_fix_duration_seconds;
				spsws_ctx.spsws_geoloc_timeout_flag = 1;
			}
			IWDG_Reload();
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.spsws_sfx_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.spsws_sfx_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.spsws_sigfox_geoloc_data.raw_frame, ((spsws_ctx.spsws_geoloc_timeout_flag) ? SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH : SPSWS_SIGFOX_GEOLOC_DATA_LENGTH), spsws_ctx.spsws_sfx_downlink_data, 2, 0);
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
			// Enter sleep mode.
			spsws_ctx.spsws_state = SPSWS_STATE_SLEEP;
			break;
		// SLEEP.
		case SPSWS_STATE_SLEEP:
			IWDG_Reload();
			// Enter sleep mode.
#ifdef IM
			PWR_EnterStopMode();
#else
			PWR_EnterLowPowerSleepMode();
#endif
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
	// Init GPIOs.
	GPIO_Init(); // Required for clock tree configuration.
	EXTI_Init(); // Required to clear RTC flags (EXTI 17).
	// Init clock and power modules.
	RCC_Init();
	PWR_Init();
	// Init clocks.
	RCC_Init();
	RCC_EnableGpio();
	RTC_Reset();
	// Low speed oscillators.
	RCC_EnableLsi();
	spsws_ctx.spsws_lse_running = RCC_EnableLse();
	// High speed oscillator.
	if (RCC_SwitchToHse() == 0) {
		RCC_SwitchToHsi();
	}
	RCC_GetLsiFrequency(&spsws_ctx.spsws_lsi_frequency_hz);
	RTC_Init(&spsws_ctx.spsws_lse_running, spsws_ctx.spsws_lsi_frequency_hz);
	// Timers.
	LPTIM1_Init();
	// Analog.
	ADC1_Init();
	// Communication interfaces.
	LPUART1_Init(spsws_ctx.spsws_lse_running);
	I2C1_Init();
	SPI1_Init();
#ifdef HW2_0
	SPI2_Init();
#endif
#ifdef HW1_0
	USART2_Init();
#endif
#ifdef HW2_0
	USART1_Init();
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
