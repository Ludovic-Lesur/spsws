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
#include "rain.h"
#include "sht3x.h"
#include "si1133.h"
#include "sky13317.h"
#include "sx1232.h"
#include "sigfox_types.h"
#include "wind.h"
// Utils.
#include "math.h"
// Applicative.
#include "at.h"
#include "error.h"
#include "mode.h"
#include "sigfox_api.h"
#include "version.h"

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
// Sigfox payload lengths.
#define SPSWS_SIGFOX_SW_VERSION_DATA_LENGTH			7
#define SPSWS_SIGFOX_ERROR_DATA_LENGTH				12
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
	SPSWS_STATE_SLEEP,
	SPSWS_STATE_LAST
} SPSWS_state_t;

typedef union {
	unsigned char raw_byte;
	struct {
		unsigned daily_downlink : 1;
		unsigned daily_geoloc : 1;
		unsigned daily_rtc_calibration : 1;
		unsigned first_rtc_calibration : 1;
		unsigned lse_status : 1;
		unsigned lsi_status : 1;
		unsigned mcu_clock_source : 1;
		unsigned station_mode : 1;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_status_t;

typedef union {
	struct {
		unsigned por : 1;
		unsigned hour_changed : 1;
		unsigned day_changed : 1;
		unsigned is_afternoon : 1;
		unsigned geoloc_timeout : 1;
	};
	unsigned char all;
} SPSWS_flags_t;

// Sigfox SW version frame data format.
typedef union {
	unsigned char raw_frame[SPSWS_SIGFOX_SW_VERSION_DATA_LENGTH];
	struct {
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_sigfox_sw_version_data_t;

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
} SPSWS_sigfox_weather_data_t;

// Sigfox monitoring frame data format.
typedef union {
	unsigned char raw_frame[SPSWS_SIGFOX_MONITORING_DATA_LENGTH];
	struct {
		unsigned tmcu_degrees : 8;
		unsigned tpcb_degrees : 8;
		unsigned hpcb_percent : 8;
		unsigned vsrc_mv : 16;
		unsigned vcap_mv : 12;
		unsigned vmcu_mv : 12;
		unsigned status : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_sigfox_monitoring_data_t;

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
} SPSWS_sigfox_geoloc_data_t;

typedef struct {
	// Global.
	SPSWS_state_t state;
	SPSWS_flags_t flags;
	// Clocks.
	unsigned int lsi_frequency_hz;
	unsigned char lse_running;
	// Wake-up management.
	RTC_time_t current_timestamp;
	RTC_time_t previous_wake_up_timestamp;
	// SW version.
	SPSWS_sigfox_sw_version_data_t sigfox_sw_version_data;
	// Monitoring.
	SPSWS_status_t status;
	SPSWS_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Weather data.
	SPSWS_sigfox_weather_data_t sigfox_weather_data;
	// Geoloc.
	NEOM8N_position_t position;
	unsigned int geoloc_fix_duration_seconds;
	SPSWS_sigfox_geoloc_data_t sigfox_geoloc_data;
	// Sigfox.
	sfx_rc_t sigfox_rc;
	sfx_u32 sigfox_rc_std_config[SIGFOX_RC_STD_CONFIG_SIZE];
	unsigned char sigfox_downlink_data[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
} SPSWS_context_t;

/*** SPSWS global variables ***/

static SPSWS_context_t spsws_ctx;

/*** SPSWS local functions ***/

#ifdef DEBUG
/* MAKE THE LED BLINK.
 * @param number_of_blinks:	Number of blinks.
 * @return:					None.
 */
void SPSWS_blink_led(unsigned char number_of_blinks) {
	unsigned char j = 0;
	unsigned int k = 0;
	for (j=0 ; j<number_of_blinks ; j++) {
		GPIO_write(&GPIO_LED, 1);
		for (k=0 ; k<20000 ; k++) {
			// Poll a bit always read as '0'.
			// This is required to avoid for loop removal by compiler (size optimization for HW1.0).
			if (((RCC -> CR) & (0b1 << 24)) != 0) {
				break;
			}
		}
		GPIO_write(&GPIO_LED, 0);
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
void SPSWS_update_time_flags(void) {
	// Retrieve current timestamp from RTC.
	RTC_get_timestamp(&spsws_ctx.current_timestamp);
	// Retrieve previous wake-up timestamp from NVM.
	unsigned char nvm_byte = 0;
	NVM_enable();
	NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), &nvm_byte);
	spsws_ctx.previous_wake_up_timestamp.year = (nvm_byte << 8);
	NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), &nvm_byte);
	spsws_ctx.previous_wake_up_timestamp.year |= nvm_byte;
	NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, &spsws_ctx.previous_wake_up_timestamp.month);
	NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, &spsws_ctx.previous_wake_up_timestamp.date);
	NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, &spsws_ctx.previous_wake_up_timestamp.hours);
	NVM_disable();
	// Check timestamp are differents (avoiding false wake-up due to RTC recalibration).
	if ((spsws_ctx.current_timestamp.year != spsws_ctx.previous_wake_up_timestamp.year) ||
		(spsws_ctx.current_timestamp.month != spsws_ctx.previous_wake_up_timestamp.month) ||
		(spsws_ctx.current_timestamp.date != spsws_ctx.previous_wake_up_timestamp.date)) {
		// Day (and thus hour) has changed.
		spsws_ctx.flags.day_changed = 1;
		spsws_ctx.flags.hour_changed = 1;
	}
	if (spsws_ctx.current_timestamp.hours != spsws_ctx.previous_wake_up_timestamp.hours) {
		// Hour as changed.
		spsws_ctx.flags.hour_changed = 1;
	}
	// Check if we are in afternoon (to enable device geolocation).
	unsigned char local_utc_offset = SPSWS_LOCAL_UTC_OFFSET_WINTER;
	if ((spsws_ctx.current_timestamp.month > SPSWS_WINTER_TIME_LAST_MONTH) && (spsws_ctx.current_timestamp.month < SPSWS_WINTER_TIME_FIRST_MONTH)) {
		local_utc_offset = SPSWS_LOCAL_UTC_OFFSET_SUMMER;
	}
	signed char local_hour = (spsws_ctx.current_timestamp.hours + local_utc_offset) % SPSWS_NUMBER_OF_HOURS_PER_DAY;
	if (local_hour < 0) {
		local_hour += SPSWS_NUMBER_OF_HOURS_PER_DAY;
	}
	if (local_hour >= SPSWS_AFTERNOON_HOUR_THRESHOLD) {
		spsws_ctx.flags.is_afternoon = 1;
	}
}

/* UPDATE PREVIOUS WAKE-UP TIMESTAMP IN NVM.
 * @param:	None.
 * @return:	None.
 */
void SPSWS_update_pwut(void) {
	// Retrieve current timestamp from RTC.
	RTC_get_timestamp(&spsws_ctx.current_timestamp);
	// Update previous wake-up timestamp.
	NVM_enable();
	NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), ((spsws_ctx.current_timestamp.year & 0xFF00) >> 8));
	NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), ((spsws_ctx.current_timestamp.year & 0x00FF) >> 0));
	NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, spsws_ctx.current_timestamp.month);
	NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, spsws_ctx.current_timestamp.date);
	NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, spsws_ctx.current_timestamp.hours);
	NVM_disable();
}

/*** SPSWS main function ***/

#if (defined IM || defined CM)
/* MAIN FUNCTION FOR IM MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init GPIOs.
	GPIO_init(); // Required for clock tree configuration.
	EXTI_init(); // Required to clear RTC flags (EXTI 17).
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Init context.
	spsws_ctx.state = SPSWS_STATE_RESET;
	spsws_ctx.flags.all = 0;
	spsws_ctx.lsi_frequency_hz = 0;
	spsws_ctx.lse_running = 0;
	spsws_ctx.geoloc_fix_duration_seconds = 0;
	NVM_enable();
	NVM_read_byte(NVM_ADDRESS_STATUS, &spsws_ctx.status.raw_byte);
	NVM_disable();
#ifdef IM
	spsws_ctx.status.field.station_mode = 0;
#else
	spsws_ctx.status.field.station_mode = 1;
#endif
	// Local variables.
	unsigned char idx = 0;
	signed char temperature = 0;
	unsigned char generic_data_u8 = 0;
	unsigned int generic_data_u32_1 = 0;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	sfx_error_t sfx_error = SFX_ERR_NONE;
#ifdef CM
	unsigned int generic_data_u32_2 = 0;
#endif
	// Set Sigfox RC.
	spsws_ctx.sigfox_rc = (sfx_rc_t) RC1;
	for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) spsws_ctx.sigfox_rc_std_config[idx] = 0;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.state) {
		// RESET.
		case SPSWS_STATE_RESET:
			IWDG_reload();
			// Check reset reason.
			if (((RCC -> CSR) & (0b1111 << 26)) != 0) {
				// IWDG, SW, NRST or POR reset: directly go to INIT state.
				spsws_ctx.flags.por = 1;
				spsws_ctx.state = SPSWS_STATE_INIT;
			}
			else {
				// Other reset or RTC wake-up: standard flow.
				spsws_ctx.state = SPSWS_STATE_HOUR_CHECK;
			}
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23); // RMVF='1'.
			break;
		// HOUR CHECK.
		case SPSWS_STATE_HOUR_CHECK:
			// Update flags.
			SPSWS_update_time_flags();
			// Check flag.
			if (spsws_ctx.flags.hour_changed == 0) {
				// False detection due to RTC recalibration.
				spsws_ctx.state = SPSWS_STATE_OFF;
			}
			else {
				// Valid wake-up.
				spsws_ctx.state = SPSWS_STATE_NVM_WAKE_UP_UPDATE;
				spsws_ctx.flags.hour_changed = 0; // Reset flag.
			}
			break;
		// NVM WAKE UP UPDATE.
		case SPSWS_STATE_NVM_WAKE_UP_UPDATE:
			// Update previous wake-up timestamp.
			SPSWS_update_pwut();
			// Check if day changed.
			if (spsws_ctx.flags.day_changed != 0) {
				// Reset daily flags.
				spsws_ctx.status.field.daily_rtc_calibration = 0;
				spsws_ctx.status.field.daily_geoloc = 0;
				spsws_ctx.status.field.daily_downlink = 0;
				// Reset flags.
				spsws_ctx.flags.day_changed = 0;
				spsws_ctx.flags.hour_changed = 0;
				spsws_ctx.flags.is_afternoon = 0;
			}
			// Go to INIT state.
			spsws_ctx.state = SPSWS_STATE_INIT;
			break;
		// INIT.
		case SPSWS_STATE_INIT:
			// Low speed oscillators and watchdog (only at POR).
			if (spsws_ctx.flags.por != 0) {
				// Start independant watchdog.
#ifndef DEBUG
				IWDG_init();
#endif
				// Reset RTC before starting oscillators.
				RTC_reset();
				spsws_ctx.status.field.lsi_status = RCC_enable_lsi();
				spsws_ctx.status.field.lse_status = RCC_enable_lse();
			}
			// High speed oscillator.
			IWDG_reload();
			RCC_enable_gpio();
			spsws_ctx.status.field.mcu_clock_source = RCC_switch_to_hse();
			if (spsws_ctx.status.field.mcu_clock_source == 0) {
				RCC_switch_to_hsi();
			}
			// Get LSI effective frequency (must be called after HSx initialization and before RTC inititialization).
			RCC_get_lsi_frequency(&spsws_ctx.lsi_frequency_hz);
			IWDG_reload();
			// Timers.
			LPTIM1_init(spsws_ctx.lsi_frequency_hz);
			// RTC (only at POR).
			if (spsws_ctx.flags.por != 0) {
				spsws_ctx.lse_running = spsws_ctx.status.field.lse_status;
				RTC_init(&spsws_ctx.lse_running, spsws_ctx.lsi_frequency_hz);
				// Update LSE status if RTC failed to start on it.
				if (spsws_ctx.lse_running == 0) {
					spsws_ctx.status.field.lse_status = 0;
				}
			}
			IWDG_reload();
			// Communication interfaces.
#ifdef HW1_0
			USART2_init();
#endif
#ifdef HW2_0
			USART1_init();
#endif
			LPUART1_init(spsws_ctx.lse_running);
			I2C1_init();
			SPI1_init();
#ifdef HW2_0
			SPI2_init();
#endif
			// Init components.
			SX1232_init();
			SX1232_tcxo(1);
			SKY13317_init();
			NEOM8N_init();
			MAX11136_init();
#ifdef CM
			WIND_init();
			RAIN_init();
#endif
			// Compute next state.
			if (spsws_ctx.flags.por == 0) {
				spsws_ctx.state = SPSWS_STATE_MEASURE;
			}
			else {
				spsws_ctx.state = SPSWS_STATE_POR;
			}
			break;
		// STATIC MEASURE.
		case SPSWS_STATE_MEASURE:
			IWDG_reload();
			// Retrieve internal ADC data.
			ADC1_init();
			ADC1_perform_measurements();
			ADC1_disable();
			ADC1_get_tmcu(&temperature);
			MATH_one_complement(temperature, 7, &generic_data_u32_1);
			spsws_ctx.sigfox_monitoring_data.field.tmcu_degrees = (unsigned char) generic_data_u32_1;
			ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &generic_data_u32_1);
			spsws_ctx.sigfox_monitoring_data.field.vmcu_mv = generic_data_u32_1;
			// Retrieve external ADC data.
#ifdef HW1_0
			SPI1_power_on();
#endif
#ifdef HW2_0
			I2C1_power_on(); // Must be called before ADC since LDR is on the MSM module (powered by I2C sensors supply).
			SPI2_power_on();
#endif
			IWDG_reload();
			MAX11136_perform_measurements();

#ifdef HW1_0
			SPI1_power_off();
#endif
#ifdef HW2_0
			SPI2_power_off();
#endif
			// Convert channel results to mV.
			MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &generic_data_u32_1);
			spsws_ctx.sigfox_monitoring_data.field.vsrc_mv = (unsigned short) generic_data_u32_1;
			MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &generic_data_u32_1);
			spsws_ctx.sigfox_monitoring_data.field.vcap_mv = (unsigned short) generic_data_u32_1;
			MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &generic_data_u32_1);
			spsws_ctx.sigfox_weather_data.field.light_percent = (unsigned char) generic_data_u32_1;
			// Retrieve weather sensors data.
#ifdef HW1_0
			I2C1_power_on();
#endif
			// Internal temperature/humidity sensor.
			IWDG_reload();
			SHT3X_perform_measurements(SHT3X_INTERNAL_I2C_ADDRESS);
			SHT3X_get_temperature(&temperature);
			MATH_one_complement(temperature, 7, &generic_data_u32_1);
			spsws_ctx.sigfox_monitoring_data.field.tpcb_degrees = (unsigned char) generic_data_u32_1;
			SHT3X_get_humidity(&generic_data_u8);
			spsws_ctx.sigfox_monitoring_data.field.hpcb_percent = generic_data_u8;
			// External temperature/humidity sensor.
#ifdef HW1_0
			spsws_ctx.sigfox_weather_data.field.temperature_degrees = spsws_ctx.sigfox_monitoring_data.field.tpcb_degrees;
			spsws_ctx.sigfox_weather_data.field.humidity_percent = spsws_ctx.sigfox_monitoring_data.field.hpcb_percent;
#endif
#ifdef HW2_0
			IWDG_reload();
			SHT3X_perform_measurements(SHT3X_EXTERNAL_I2C_ADDRESS);
			SHT3X_get_temperature(&temperature);
			MATH_one_complement(temperature, 7, &generic_data_u32_1);
			spsws_ctx.sigfox_weather_data.field.temperature_degrees = (unsigned char) generic_data_u32_1;
			SHT3X_get_humidity(&generic_data_u8);
			spsws_ctx.sigfox_weather_data.field.humidity_percent = generic_data_u8;
#endif
			// External pressure/temperature sensor.
			IWDG_reload();
			DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_get_pressure(&generic_data_u32_1);
			// TODO add DPS310_PRESSURE_ERROR_VALUE macro in main and check DPS310 status
			spsws_ctx.sigfox_weather_data.field.absolute_pressure_tenth_hpa = (generic_data_u32_1 / 10);
			// External UV index sensor.
			IWDG_reload();
			SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_get_uv_index(&generic_data_u8);
			spsws_ctx.sigfox_weather_data.field.uv_index = generic_data_u8;
			// Turn sensors off.
			I2C1_power_off();
#ifdef CM
			IWDG_reload();
			// Retrieve wind measurements.
			WIND_get_speed(&generic_data_u32_1, &generic_data_u32_2);
			spsws_ctx.sigfox_weather_data.field.average_wind_speed_kmh = (generic_data_u32_1 / 1000);
			spsws_ctx.sigfox_weather_data.field.peak_wind_speed_kmh = (generic_data_u32_2 / 1000);
			WIND_get_direction(&generic_data_u32_1);
			// TODO add WIND_DIRECTION_ERROR_VALUE 0xFF macro in main and check WIND status
			spsws_ctx.sigfox_weather_data.field.average_wind_direction_two_degrees = (generic_data_u32_1 / 2);
			// Retrieve rain measurements.
			RAIN_get_pluviometry(&generic_data_u8);
			spsws_ctx.sigfox_weather_data.field.rain_mm = generic_data_u8;
#endif
			// Read status byte.
			spsws_ctx.sigfox_monitoring_data.field.status = spsws_ctx.status.raw_byte;
			// Compute next state.
			spsws_ctx.state = SPSWS_STATE_MONITORING;
			break;
		// MONITORING.
		case SPSWS_STATE_MONITORING:
			IWDG_reload();
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.sigfox_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.sigfox_monitoring_data.raw_frame, SPSWS_SIGFOX_MONITORING_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			spsws_ctx.state = SPSWS_STATE_WEATHER_DATA;
			break;
		// WEATHER DATA.
		case SPSWS_STATE_WEATHER_DATA:
			IWDG_reload();
			// Send uplink weather frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.sigfox_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.sigfox_weather_data.raw_frame, SPSWS_SIGFOX_WEATHER_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			if (spsws_ctx.status.field.daily_rtc_calibration == 0) {
				// Perform RTC calibration.
				spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
			}
			else {
				if ((spsws_ctx.status.field.daily_geoloc == 0) && (spsws_ctx.flags.is_afternoon != 0)) {
					// Perform device geolocation.
					spsws_ctx.state = SPSWS_STATE_GEOLOC;
				}
				else {
					// Enter stop mode.
					spsws_ctx.state = SPSWS_STATE_OFF;
				}
			}
			break;
		// POR.
		case SPSWS_STATE_POR:
			IWDG_reload();
			// Build software version;
			spsws_ctx.sigfox_sw_version_data.field.major_version = GIT_MAJOR_VERSION;
			spsws_ctx.sigfox_sw_version_data.field.minor_version = GIT_MINOR_VERSION;
			spsws_ctx.sigfox_sw_version_data.field.commit_index = GIT_COMMIT_INDEX;
			spsws_ctx.sigfox_sw_version_data.field.commit_id = GIT_COMMIT_ID;
			spsws_ctx.sigfox_sw_version_data.field.dirty_flag = GIT_DIRTY_FLAG;
			// Send SW version frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.sigfox_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.sigfox_sw_version_data.raw_frame, SPSWS_SIGFOX_SW_VERSION_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset all daily flags.
			spsws_ctx.status.field.first_rtc_calibration = 0;
			spsws_ctx.status.field.daily_rtc_calibration = 0;
			spsws_ctx.status.field.daily_geoloc = 0;
			spsws_ctx.status.field.daily_downlink = 0;
			// Perform first RTC calibration.
			spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
			break;
		// GEOLOC.
		case SPSWS_STATE_GEOLOC:
			IWDG_reload();
			// Get position from GPS.
			LPUART1_power_on();
			neom8n_status = NEOM8N_get_position(&spsws_ctx.position, SPSWS_GEOLOC_TIMEOUT_SECONDS, &spsws_ctx.geoloc_fix_duration_seconds);
			LPUART1_power_off();
			// Update flag whatever the result.
			spsws_ctx.status.field.daily_geoloc = 1;
			// Parse result.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Build frame.
				spsws_ctx.sigfox_geoloc_data.field.latitude_degrees = spsws_ctx.position.lat_degrees;
				spsws_ctx.sigfox_geoloc_data.field.latitude_minutes = spsws_ctx.position.lat_minutes;
				spsws_ctx.sigfox_geoloc_data.field.latitude_seconds = spsws_ctx.position.lat_seconds;
				spsws_ctx.sigfox_geoloc_data.field.latitude_north_flag = spsws_ctx.position.lat_north_flag;
				spsws_ctx.sigfox_geoloc_data.field.longitude_degrees = spsws_ctx.position.long_degrees;
				spsws_ctx.sigfox_geoloc_data.field.longitude_minutes = spsws_ctx.position.long_minutes;
				spsws_ctx.sigfox_geoloc_data.field.longitude_seconds = spsws_ctx.position.long_seconds;
				spsws_ctx.sigfox_geoloc_data.field.longitude_east_flag = spsws_ctx.position.long_east_flag;
				spsws_ctx.sigfox_geoloc_data.field.altitude_meters = spsws_ctx.position.altitude;
				spsws_ctx.sigfox_geoloc_data.field.gps_fix_duration = spsws_ctx.geoloc_fix_duration_seconds;
			}
			else {
				spsws_ctx.sigfox_geoloc_data.raw_frame[0] = spsws_ctx.geoloc_fix_duration_seconds;
				spsws_ctx.flags.geoloc_timeout = 1;
			}
			IWDG_reload();
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_set_std_config(spsws_ctx.sigfox_rc_std_config, SFX_FALSE);
				sfx_error = SIGFOX_API_send_frame(spsws_ctx.sigfox_geoloc_data.raw_frame, ((spsws_ctx.flags.geoloc_timeout) ? SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH : SPSWS_SIGFOX_GEOLOC_DATA_LENGTH), spsws_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset geoloc variables.
			spsws_ctx.flags.geoloc_timeout = 0;
			spsws_ctx.geoloc_fix_duration_seconds = 0;
			// Enter standby mode.
			spsws_ctx.state = SPSWS_STATE_OFF;
			break;
		// RTC CALIBRATION.
		case SPSWS_STATE_RTC_CALIBRATION:
			IWDG_reload();
			// Turn radio TCXO off since Sigfox is not required anymore.
			SX1232_tcxo(0);
			// Get current timestamp from GPS.{
			LPUART1_power_on();
			neom8n_status = NEOM8N_get_time(&spsws_ctx.current_timestamp, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS, 0);
			LPUART1_power_off();
			// Calibrate RTC if timestamp is available.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Update RTC registers.
				RTC_calibrate(&spsws_ctx.current_timestamp);
				// Update PWUT when first calibration.
				if (spsws_ctx.status.field.first_rtc_calibration == 0) {
					SPSWS_update_pwut();
				}
				// Update calibration flags.
				spsws_ctx.status.field.first_rtc_calibration = 1;
				spsws_ctx.status.field.daily_rtc_calibration = 1;
			}
			// Enter standby mode.
			spsws_ctx.state = SPSWS_STATE_OFF;
			break;
		// OFF.
		case SPSWS_STATE_OFF:
			IWDG_reload();
			// Clear POR flag.
			spsws_ctx.flags.por = 0;
			// Turn peripherals off.
			SX1232_disable();
			SKY13317_disable();
			MAX11136_disable();
			SX1232_tcxo(0);
#ifdef HW2_0
			SPI2_disable();
#endif
			TIM2_disable();
			LPTIM1_disable();
			SPI1_disable();
			LPUART1_disable();
			I2C1_disable();
			// Store status byte in NVM.
			NVM_enable();
			NVM_write_byte(NVM_ADDRESS_STATUS, spsws_ctx.status.raw_byte);
			NVM_disable();
			// Switch to internal MSI 65kHz (must be called before WIND functions to init LPTIM with right clock frequency).
			RCC_switch_to_msi();
			RCC_disable_gpio();
#ifdef CM
			// Re-start continuous measurements.
			WIND_reset_data();
			RAIN_reset_data();
			WIND_start_continuous_measure();
			RAIN_start_continuous_measure();
#endif
			// Clear RTC flags.
			RTC_clear_alarm_a_flag();
			RTC_clear_alarm_b_flag();
			RTC_enable_alarm_a_interrupt();
			RTC_enable_alarm_b_interrupt();
			// Enter sleep mode.
			spsws_ctx.state = SPSWS_STATE_SLEEP;
			break;
		// SLEEP.
		case SPSWS_STATE_SLEEP:
			IWDG_reload();
			// Enter sleep mode.
			PWR_enter_stop_mode();
			// Check RTC flags.
			if (RTC_get_alarm_b_flag() != 0) {
#ifdef CM
				// Call WIND callback.
				WIND_measurement_period_callback();
#endif
				// Clear RTC flags.
				RTC_clear_alarm_b_flag();
			}
			if (RTC_get_alarm_a_flag() != 0) {
#ifdef CM
				// Stop continuous measurements.
				WIND_stop_continuous_measure();
				RAIN_stop_continuous_measure();
#endif
				// Disable RTC alarm interrupts.
				RTC_disable_alarm_a_interrupt();
				RTC_disable_alarm_b_interrupt();
				// Clear RTC flags.
				RTC_clear_alarm_a_flag();
				// Wake-up.
				spsws_ctx.state = SPSWS_STATE_RESET;
			}
			break;
		// UNKNOWN STATE.
		default:
			IWDG_reload();
			// Enter standby mode.
			spsws_ctx.state = SPSWS_STATE_OFF;
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
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init GPIOs.
	GPIO_init(); // Required for clock tree configuration.
	EXTI_init(); // Required to clear RTC flags (EXTI 17).
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Init clocks.
	RCC_init();
	RCC_enable_gpio();
	RTC_reset();
	// Low speed oscillators.
	RCC_enable_lsi();
	spsws_ctx.lse_running = RCC_enable_lse();
	// High speed oscillator.
	if (RCC_switch_to_hse() == 0) {
		RCC_switch_to_hsi();
	}
	RCC_get_lsi_frequency(&spsws_ctx.lsi_frequency_hz);
	RTC_init(&spsws_ctx.lse_running, spsws_ctx.lsi_frequency_hz);
	// Timers.
	LPTIM1_init(spsws_ctx.lsi_frequency_hz);
	// Analog.
	ADC1_init();
	// Communication interfaces.
	LPUART1_init(spsws_ctx.lse_running);
	I2C1_init();
	SPI1_init();
#ifdef HW2_0
	SPI2_init();
#endif
#ifdef HW1_0
	USART2_init();
#endif
#ifdef HW2_0
	USART1_init();
#endif
	// Init components.
	SX1232_init();
	SX1232_tcxo(1);
	SKY13317_init();
	NEOM8N_init();
	MAX11136_init();
	WIND_init();
	RAIN_init();
	// Init applicative layers.
	AT_init();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		PWR_enter_sleep_mode();
		// Wake-up: perform AT command parsing.
		AT_task();
	}
	return 0;
}
#endif
