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
// Utils.
#include "error.h"
#include "math.h"
#include "types.h"
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
// Sigfox.
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
// Applicative.
#include "at.h"
#include "error_base.h"
#include "mode.h"
#include "version.h"

/*** SPSWS macros ***/

// Timeouts.
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS		180
#define SPSWS_GEOLOC_TIMEOUT_SECONDS				120
// GPS altitude stability filter
#define SPSWS_GPS_ALTITUDE_STABILITY_FILTER			5
// Sigfox UL payloads size.
#define SPSWS_SIGFOX_STARTUP_DATA_SIZE				8
#define SPSWS_SIGFOX_ERROR_DATA_SIZE				12
#if (defined SPSWS_WIND_MEASUREMENT) || (defined SPSWS_RAIN_MEASUREMENT)
#define SPSWS_SIGFOX_WEATHER_DATA_SIZE				10
#else
#define SPSWS_SIGFOX_WEATHER_DATA_SIZE				6
#endif
#define SPSWS_SIGFOX_MONITORING_DATA_SIZE			9
#define SPSWS_SIGFOX_GEOLOC_DATA_SIZE				11
#define SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE		3
#define SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE			12
#ifdef SPSWS_FLOOD_MEASUREMENT
#define SPSWS_SIGFOX_FLOOD_DATA_SIZE				2
#define SPSWS_FLOOD_LEVEL_CHANGE_THRESHOLD			10 // Number of consecutive reading to validate a new flood level.
#endif
// Error values.
#define SPSWS_ERROR_VALUE_ANALOG_12BITS				0xFFF
#define SPSWS_ERROR_VALUE_ANALOG_16BITS				0xFFFF
#define SPSWS_ERROR_VALUE_LIGHT						0xFF
#define SPSWS_ERROR_VALUE_TEMPERATURE				0x7F
#define SPSWS_ERROR_VALUE_HUMIDITY					0xFF
#define SPSWS_ERROR_VALUE_UV_INDEX					0xFF
#define SPSWS_ERROR_VALUE_PRESSURE					0xFFFF
#define SPSWS_ERROR_VALUE_WIND						0xFF
#define SPSWS_ERROR_VALUE_RAIN						0xFF
// Measurements buffers length.
#define MEASUREMENT_PERIOD_SECONDS					60
#define MEASUREMENT_BUFFER_LENGTH					(3600 / MEASUREMENT_PERIOD_SECONDS)

/*** SPSWS structures ***/

/*******************************************************************/
typedef enum {
	SPSWS_STATE_STARTUP,
	SPSWS_STATE_WAKEUP,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
#ifdef SPSWS_FLOOD_MEASUREMENT
	SPSWS_STATE_FLOOD_ALARM,
#endif
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_ERROR_STACK,
	SPSWS_STATE_OFF,
	SPSWS_STATE_SLEEP,
	SPSWS_STATE_LAST
} SPSWS_state_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned daily_downlink : 1;
		unsigned daily_geoloc : 1;
		unsigned daily_rtc_calibration : 1;
		unsigned first_rtc_calibration : 1;
		unsigned lse_status : 1;
		unsigned lsi_status : 1;
		unsigned mcu_clock_source : 1;
		unsigned station_mode : 1;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} SPSWS_status_t;

/*******************************************************************/
typedef union {
	struct {
#ifdef SPSWS_FLOOD_MEASUREMENT
		unsigned unused : 1;
		unsigned flood_alarm : 1;
#else
		unsigned unused : 2;
#endif
		unsigned fixed_hour_alarm : 1;
		unsigned wake_up : 1;
		unsigned is_afternoon : 1;
		unsigned day_changed : 1;
		unsigned hour_changed : 1;
		unsigned por : 1;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} SPSWS_flags_t;

/*******************************************************************/
typedef struct {
	uint8_t data[MEASUREMENT_BUFFER_LENGTH];
	uint8_t count;
	uint8_t full_flag;
} SPSWS_measurement_u8_t;

/*******************************************************************/
typedef struct {
	uint16_t data[MEASUREMENT_BUFFER_LENGTH];
	uint8_t count;
	uint8_t full_flag;
} SPSWS_measurement_u16_t;

/*******************************************************************/
typedef struct {
	SPSWS_measurement_u8_t tamb_degrees;
	SPSWS_measurement_u8_t hamb_percent;
	SPSWS_measurement_u8_t light_percent;
	SPSWS_measurement_u8_t uv_index;
	SPSWS_measurement_u16_t patm_abs_tenth_hpa;
	SPSWS_measurement_u8_t tmcu_degrees;
	SPSWS_measurement_u8_t tpcb_degrees;
	SPSWS_measurement_u8_t hpcb_percent;
	SPSWS_measurement_u16_t vsrc_mv;
	SPSWS_measurement_u16_t vmcu_mv;
	// Note: VCAP is not averaged since the last value is the most relevant.
} SPSWS_measurements_t;

/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_STARTUP_DATA_SIZE];
	struct {
		unsigned reset_reason : 8;
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_startup_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_WEATHER_DATA_SIZE];
	struct {
		unsigned tamb_degrees : 8;
		unsigned hamb_percent : 8;
		unsigned light_percent : 8;
		unsigned uv_index : 8;
		unsigned patm_abs_tenth_hpa : 16;
#if (defined SPSWS_WIND_MEASUREMENT) || (defined SPSWS_RAIN_MEASUREMENT)
		unsigned wind_speed_average_kmh : 8;
		unsigned wind_speed_peak_kmh : 8;
		unsigned wind_direction_average_two_degrees : 8;
		unsigned rain_mm : 8;
#endif
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_weather_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_MONITORING_DATA_SIZE];
	struct {
		unsigned tmcu_degrees : 8;
		unsigned tpcb_degrees : 8;
		unsigned hpcb_percent : 8;
		unsigned vsrc_mv : 16;
		unsigned vcap_mv : 12;
		unsigned vmcu_mv : 12;
		unsigned status : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_monitoring_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_GEOLOC_DATA_SIZE];
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
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_geoloc_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE];
	struct {
		unsigned error_code : 16;
		unsigned fix_duration_seconds : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_geoloc_timeout_data_t;

#ifdef SPSWS_FLOOD_MEASUREMENT
/*******************************************************************/
typedef union {
	uint8_t frame[SPSWS_SIGFOX_FLOOD_DATA_SIZE];
	struct {
		unsigned level : 8;
		unsigned rain_mm : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));;
} SPSWS_sigfox_flood_data_t;
#endif

/*******************************************************************/
typedef struct {
	// Global.
	SPSWS_state_t state;
	SPSWS_flags_t flags;
	// Wake-up management.
	RTC_time_t current_time;
	RTC_time_t previous_wake_up_time;
	// Measurements buffers.
	SPSWS_measurements_t measurements;
	uint32_t seconds_counter;
	// SW version.
	SPSWS_sigfox_startup_data_t sigfox_startup_data;
	// Monitoring.
	SPSWS_status_t status;
	SPSWS_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Weather data.
	SPSWS_sigfox_weather_data_t sigfox_weather_data;
#ifdef SPSWS_FLOOD_MEASUREMENT
	uint8_t flood_level;
	uint8_t flood_level_change_count;
	SPSWS_sigfox_flood_data_t sigfox_flood_data;
#endif
	// Geoloc.
	NEOM8N_position_t position;
	uint32_t geoloc_fix_duration_seconds;
	SPSWS_sigfox_geoloc_data_t sigfox_geoloc_data;
	SPSWS_sigfox_geoloc_timeout_data_t sigfox_geoloc_timeout_data;
	// Error stack.
	uint8_t sigfox_error_stack_data[SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE];
} SPSWS_context_t;

/*** SPSWS global variables ***/

static SPSWS_context_t spsws_ctx;

/*** SPSWS local functions ***/

/*******************************************************************/
static void _SPSWS_reset_measurements(void) {
	// Weather data
	spsws_ctx.measurements.tamb_degrees.count = 0;
	spsws_ctx.measurements.tamb_degrees.full_flag = 0;
	spsws_ctx.measurements.hamb_percent.count = 0;
	spsws_ctx.measurements.hamb_percent.full_flag = 0;
	spsws_ctx.measurements.light_percent.count = 0;
	spsws_ctx.measurements.light_percent.full_flag = 0;
	spsws_ctx.measurements.uv_index.count = 0;
	spsws_ctx.measurements.uv_index.full_flag = 0;
	spsws_ctx.measurements.patm_abs_tenth_hpa.count = 0;
	spsws_ctx.measurements.patm_abs_tenth_hpa.full_flag = 0;
#ifdef SPSWS_WIND_MEASUREMENT
	WIND_reset_data();
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
	RAIN_reset_rainfall();
#endif
	// Monitoring data.
	spsws_ctx.measurements.tmcu_degrees.count = 0;
	spsws_ctx.measurements.tmcu_degrees.full_flag = 0;
	spsws_ctx.measurements.tpcb_degrees.count = 0;
	spsws_ctx.measurements.tpcb_degrees.full_flag = 0;
	spsws_ctx.measurements.hpcb_percent.count = 0;
	spsws_ctx.measurements.hpcb_percent.full_flag = 0;
	spsws_ctx.measurements.vsrc_mv.count = 0;
	spsws_ctx.measurements.vsrc_mv.full_flag = 0;
	spsws_ctx.measurements.vmcu_mv.count = 0;
	spsws_ctx.measurements.vmcu_mv.full_flag = 0;
}

/*******************************************************************/
static void _SPSWS_init_context(void) {
	// Init context.
	spsws_ctx.state = SPSWS_STATE_STARTUP;
	spsws_ctx.flags.all = 0;
	spsws_ctx.flags.por = 1;
	spsws_ctx.geoloc_fix_duration_seconds = 0;
	spsws_ctx.seconds_counter = 0;
	spsws_ctx.status.all = 0;
#ifdef SPSWS_FLOOD_MEASUREMENT
	spsws_ctx.flood_level = 0;
	spsws_ctx.flood_level_change_count = 0;
#endif
	// Init station mode.
#if (defined SPSWS_WIND_MEASUREMENT) || (defined SPSWS_RAIN_MEASUREMENT)
	spsws_ctx.status.station_mode = 0b1;
#else
	spsws_ctx.status.station_mode = 0b0;
#endif
	// Reset measurements.
	_SPSWS_reset_measurements();
}

/*******************************************************************/
static void _SPSWS_update_clocks(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint8_t clock_status = 0;
	// Calibrate clocks.
	rcc_status = RCC_calibrate();
	RCC_stack_error();
	// Update clock status.
	rcc_status = RCC_get_status(RCC_CLOCK_LSI, &clock_status);
	RCC_stack_error();
	spsws_ctx.status.lsi_status = (clock_status == 0) ? 0b0 : 0b1;
	rcc_status = RCC_get_status(RCC_CLOCK_LSE, &clock_status);
	RCC_stack_error();
	spsws_ctx.status.lse_status = (clock_status == 0) ? 0b0 : 0b1;
}

/*******************************************************************/
static void _SPSWS_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	uint8_t device_id_lsbyte = 0;
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	RCC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
#ifndef DEBUG
	// Start independent watchdog.
	iwdg_status = IWDG_init();
	IWDG_stack_error();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_hsi();
	RCC_stack_error();
	// Calibrate and update clock status.
	_SPSWS_update_clocks();
	// Read LS byte of the device ID to add a random delay in RTC alarm.
	nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES - 1), &device_id_lsbyte);
	NVM_stack_error();
	rtc_status = RTC_init(device_id_lsbyte);
	RTC_stack_error();
	// Internal.
	lptim1_status = LPTIM1_init();
	LPTIM1_stack_error();
	// Init continuous measurements drivers.
	POWER_init();
#ifdef SPSWS_WIND_MEASUREMENT
	WIND_init();
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
	RAIN_init();
#endif
	// Init LED pin.
	GPIO_configure(&GPIO_LED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*******************************************************************/
#define _SPSWS_increment_measurement_count(measurement_struct) { \
	/* Increment index */ \
	measurement_struct.count++; \
	/* Manage rollover and flag */ \
	if (measurement_struct.count >= MEASUREMENT_BUFFER_LENGTH) { \
		measurement_struct.count = 0; \
		measurement_struct.full_flag = 1; \
	} \
}

#ifndef ATM
/*******************************************************************/
static void _SPSWS_update_time_flags(void) {
	// Local variables.
	RTC_status_t rtc_status = RTC_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t nvm_byte = 0;
	uint8_t local_utc_offset = 0;
	int8_t local_hour = 0;
	// Retrieve current time from RTC.
	rtc_status = RTC_get_time(&spsws_ctx.current_time);
	RTC_stack_error();
	// Retrieve previous wake-up time from NVM.
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), &nvm_byte);
	NVM_stack_error();
	spsws_ctx.previous_wake_up_time.year = (nvm_byte << 8);
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), &nvm_byte);
	NVM_stack_error();
	spsws_ctx.previous_wake_up_time.year |= nvm_byte;
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, &spsws_ctx.previous_wake_up_time.month);
	NVM_stack_error();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, &spsws_ctx.previous_wake_up_time.date);
	NVM_stack_error();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, &spsws_ctx.previous_wake_up_time.hours);
	NVM_stack_error();
	// Check time are different (avoiding false wake-up due to RTC calibration).
	if ((spsws_ctx.current_time.year != spsws_ctx.previous_wake_up_time.year) ||
		(spsws_ctx.current_time.month != spsws_ctx.previous_wake_up_time.month) ||
		(spsws_ctx.current_time.date != spsws_ctx.previous_wake_up_time.date)) {
		// Day (and thus hour) has changed.
		spsws_ctx.flags.day_changed = 1;
		spsws_ctx.flags.hour_changed = 1;
	}
	if (spsws_ctx.current_time.hours != spsws_ctx.previous_wake_up_time.hours) {
		// Hour as changed.
		spsws_ctx.flags.hour_changed = 1;
	}
	// Check if we are in afternoon (to enable device geolocation).
	local_utc_offset = RTC_LOCAL_UTC_OFFSET_WINTER;
	if ((spsws_ctx.current_time.month > RTC_WINTER_TIME_LAST_MONTH) && (spsws_ctx.current_time.month < RTC_WINTER_TIME_FIRST_MONTH)) {
		local_utc_offset = RTC_LOCAL_UTC_OFFSET_SUMMER;
	}
	local_hour = (spsws_ctx.current_time.hours + local_utc_offset) % RTC_NUMBER_OF_HOURS_PER_DAY;
	if (local_hour < 0) {
		local_hour += RTC_NUMBER_OF_HOURS_PER_DAY;
	}
	if (local_hour >= RTC_AFTERNOON_HOUR_THRESHOLD) {
		spsws_ctx.flags.is_afternoon = 1;
	}
}
#endif

#ifndef ATM
/*******************************************************************/
static void _SPSWS_update_pwut(void) {
	// Local variables.
	RTC_status_t rtc_status = RTC_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Retrieve current time from RTC.
	rtc_status = RTC_get_time(&spsws_ctx.current_time);
	RTC_stack_error();
	// Update previous wake-up time.
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), ((spsws_ctx.current_time.year & 0xFF00) >> 8));
	NVM_stack_error();
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), ((spsws_ctx.current_time.year & 0x00FF) >> 0));
	NVM_stack_error();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, spsws_ctx.current_time.month);
	NVM_stack_error();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, spsws_ctx.current_time.date);
	NVM_stack_error();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, spsws_ctx.current_time.hours);
	NVM_stack_error();
}
#endif

#ifndef ATM
/*******************************************************************/
static void _SPSWS_compute_final_measurements(void) {
	// Local variables.
	MATH_status_t math_status = MATH_SUCCESS;
	uint8_t data_length = 0;
	uint8_t generic_u8 = 0;
	uint16_t generic_u16 = 0;
	// Temperature
	data_length = (spsws_ctx.measurements.tamb_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tamb_degrees.count;
	math_status = MATH_min_u8(spsws_ctx.measurements.tamb_degrees.data, data_length, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_weather_data.tamb_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : generic_u8;
	// Humidity.
	data_length = (spsws_ctx.measurements.hamb_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.hamb_percent.count;
	math_status = MATH_median_filter_u8(spsws_ctx.measurements.hamb_percent.data, data_length, 0, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_weather_data.hamb_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_HUMIDITY : generic_u8;
	// Light.
	data_length = (spsws_ctx.measurements.light_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.light_percent.count;
	math_status = MATH_median_filter_u8(spsws_ctx.measurements.light_percent.data, data_length, 0, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_weather_data.light_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_LIGHT : generic_u8;
	// UV index.
	data_length = (spsws_ctx.measurements.uv_index.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.uv_index.count;
	math_status = MATH_max_u8(spsws_ctx.measurements.uv_index.data, data_length, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_weather_data.uv_index = (data_length == 0) ? SPSWS_ERROR_VALUE_UV_INDEX : generic_u8;
	// Absolute pressure.
	data_length = (spsws_ctx.measurements.patm_abs_tenth_hpa.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.patm_abs_tenth_hpa.count;
	math_status = MATH_median_filter_u16(spsws_ctx.measurements.patm_abs_tenth_hpa.data, data_length, 0, &generic_u16);
	MATH_stack_error();
	spsws_ctx.sigfox_weather_data.patm_abs_tenth_hpa = (data_length == 0) ? SPSWS_ERROR_VALUE_PRESSURE : generic_u16;
	// MCU temperature.
	data_length = (spsws_ctx.measurements.tmcu_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tmcu_degrees.count;
	math_status = MATH_min_u8(spsws_ctx.measurements.tmcu_degrees.data, data_length, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_monitoring_data.tmcu_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : generic_u8;
	// PCB temperature.
	data_length = (spsws_ctx.measurements.tpcb_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tpcb_degrees.count;
	math_status = MATH_min_u8(spsws_ctx.measurements.tpcb_degrees.data, data_length, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_monitoring_data.tpcb_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : generic_u8;
	// PCB humidity.
	data_length = (spsws_ctx.measurements.hpcb_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.hpcb_percent.count;
	math_status = MATH_median_filter_u8(spsws_ctx.measurements.hpcb_percent.data, data_length, 0, &generic_u8);
	MATH_stack_error();
	spsws_ctx.sigfox_monitoring_data.hpcb_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_HUMIDITY : generic_u8;
	// Solar cell voltage.
	data_length = (spsws_ctx.measurements.vsrc_mv.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.vsrc_mv.count;
	math_status = MATH_median_filter_u16(spsws_ctx.measurements.vsrc_mv.data, data_length, 0, &generic_u16);
	MATH_stack_error();
	spsws_ctx.sigfox_monitoring_data.vsrc_mv = (data_length == 0) ? SPSWS_ERROR_VALUE_ANALOG_16BITS : generic_u16;
	// MCU voltage.
	data_length = (spsws_ctx.measurements.vmcu_mv.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.vmcu_mv.count;
	math_status = MATH_median_filter_u16(spsws_ctx.measurements.vmcu_mv.data, data_length, 0, &generic_u16);
	MATH_stack_error();
	spsws_ctx.sigfox_monitoring_data.vmcu_mv = (data_length == 0) ? SPSWS_ERROR_VALUE_ANALOG_12BITS : generic_u16;
}
#endif

#ifndef ATM
/*******************************************************************/
static void _SPSWS_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
	// Local variables.
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_API_config_t lib_config;
	// Library configuration.
	lib_config.rc = &SIGFOX_RC1;
	// Open library.
	sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
	SIGFOX_EP_API_stack_error();
	if (sigfox_ep_api_status == SIGFOX_EP_API_SUCCESS) {
		// Send message.
		sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
		SIGFOX_EP_API_stack_error();
	}
	// Close library.
	sigfox_ep_api_status = SIGFOX_EP_API_close();
	SIGFOX_EP_API_stack_error();
}
#endif

/*** SPSWS main function ***/

#ifndef ATM
/*******************************************************************/
int main (void) {
	// Init board.
	_SPSWS_init_context();
	_SPSWS_init_hw();
	// Local variables.
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	SIGFOX_EP_API_application_message_t application_message;
	ERROR_code_t error_code = 0;
	int8_t temperature = 0;
	uint8_t generic_u8 = 0;
	uint32_t generic_u32_1 = 0;
	uint8_t idx = 0;
#ifdef SPSWS_WIND_MEASUREMENT
	uint32_t generic_u32_2 = 0;
	WIND_status_t wind_status = WIND_SUCCESS;
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
	RAIN_status_t rain_status = RAIN_SUCCESS;
#endif
	// Application message default parameters.
	application_message.common_parameters.number_of_frames = 3;
	application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef BIDIRECTIONAL
	application_message.bidirectional_flag = 0;
#endif
	application_message.ul_payload = SFX_NULL;
	application_message.ul_payload_size_bytes = 0;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.state) {
		case SPSWS_STATE_STARTUP:
			IWDG_reload();
			// Fill reset reason and software version.
			spsws_ctx.sigfox_startup_data.reset_reason = ((RCC -> CSR) >> 24) & 0xFF;
			spsws_ctx.sigfox_startup_data.major_version = GIT_MAJOR_VERSION;
			spsws_ctx.sigfox_startup_data.minor_version = GIT_MINOR_VERSION;
			spsws_ctx.sigfox_startup_data.commit_index = GIT_COMMIT_INDEX;
			spsws_ctx.sigfox_startup_data.commit_id = GIT_COMMIT_ID;
			spsws_ctx.sigfox_startup_data.dirty_flag = GIT_DIRTY_FLAG;
			// Send SW version frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
			application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_startup_data.frame);
			application_message.ul_payload_size_bytes = SPSWS_SIGFOX_STARTUP_DATA_SIZE;
			_SPSWS_send_sigfox_message(&application_message);
			// Perform first RTC calibration.
			spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
			break;
		case SPSWS_STATE_WAKEUP:
			IWDG_reload();
			// Update flags.
			_SPSWS_update_time_flags();
			// Check alarm flag..
			if (spsws_ctx.flags.fixed_hour_alarm != 0) {
				// Check hour change flag.
				if (spsws_ctx.flags.hour_changed != 0) {
					// Valid fixed hour wake-up.
					_SPSWS_update_pwut();
					// Check if day changed.
					if (spsws_ctx.flags.day_changed != 0) {
						// Reset daily flags.
						spsws_ctx.status.daily_rtc_calibration = 0;
						spsws_ctx.status.daily_geoloc = 0;
						spsws_ctx.status.daily_downlink = 0;
						// Reset flags.
						spsws_ctx.flags.day_changed = 0;
						spsws_ctx.flags.hour_changed = 0;
						spsws_ctx.flags.is_afternoon = 0;
					}
					spsws_ctx.flags.hour_changed = 0; // Reset flag.
					// Calibrate and update clocks.
					_SPSWS_update_clocks();
					// Next state
					spsws_ctx.state = SPSWS_STATE_MONITORING;
				}
				else {
					// False detection due to RTC calibration.
					spsws_ctx.state = SPSWS_STATE_OFF;
				}
			}
			else {
				// Intermediate measure wake-up.
				spsws_ctx.state = SPSWS_STATE_MEASURE;
#ifdef SPSWS_FLOOD_MEASUREMENT
				// Calibrate clocks if alarm has to be sent.
				if (spsws_ctx.flags.flood_alarm != 0) {
					// Calibrate and update clocks.
					_SPSWS_update_clocks();
				}
#endif
			}
			break;
		case SPSWS_STATE_RTC_CALIBRATION:
			IWDG_reload();
			// Get current time from GPS.
			power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			neom8n_status = NEOM8N_get_time(&spsws_ctx.current_time, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS, &generic_u32_1);
			NEOM8N_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_GPS);
			POWER_stack_error();
			// Calibrate RTC if time is available.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Update RTC registers.
				rtc_status = RTC_calibrate(&spsws_ctx.current_time);
				RTC_stack_error();
				// Update PWUT when first calibration.
				if (spsws_ctx.status.first_rtc_calibration == 0) {
					_SPSWS_update_pwut();
				}
				// Update calibration flags.
				spsws_ctx.status.first_rtc_calibration = 1;
				spsws_ctx.status.daily_rtc_calibration = 1;
			}
			else {
				// In POR case, to avoid wake-up directly after RTC calibration (alarm A will occur during the first GPS time acquisition because of the RTC reset and the random delay).
				RTC_clear_alarm_a_flag();
			}
			// Send error stack at start
			spsws_ctx.state = (spsws_ctx.flags.por != 0) ? SPSWS_STATE_ERROR_STACK : SPSWS_STATE_OFF;
			break;
		case SPSWS_STATE_MEASURE:
			IWDG_reload();
			// Retrieve internal ADC data.
			power_status = POWER_enable(POWER_DOMAIN_ANALOG_INTERNAL, LPTIM_DELAY_MODE_SLEEP);
			POWER_stack_error();
			adc1_status = ADC1_perform_measurements();
			ADC1_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_ANALOG_INTERNAL);
			POWER_stack_error();
			// Check status.
			if (adc1_status == ADC_SUCCESS) {
				// Read data.
				adc1_status = ADC1_get_tmcu(&temperature);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					// Convert to 1-complement.
					math_status = MATH_int32_to_signed_magnitude(temperature, 7, &generic_u32_1);
					MATH_stack_error();
					if (math_status == MATH_SUCCESS) {
						spsws_ctx.measurements.tmcu_degrees.data[spsws_ctx.measurements.tmcu_degrees.count] = (uint8_t) generic_u32_1;
						_SPSWS_increment_measurement_count(spsws_ctx.measurements.tmcu_degrees);
					}
				}
				adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &generic_u32_1);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					spsws_ctx.measurements.vmcu_mv.data[spsws_ctx.measurements.vmcu_mv.count] = generic_u32_1;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.vmcu_mv);
				}
			}
			IWDG_reload();
			// Retrieve external ADC data.
			// Note: digital sensors power supply must also be enabled at this step to power the LDR.
			power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			power_status = POWER_enable(POWER_DOMAIN_ANALOG_EXTERNAL, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			max11136_status = MAX11136_perform_measurements();
			MAX11136_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_ANALOG_EXTERNAL);
			POWER_stack_error();
			// Check status.
			if (max11136_status == MAX11136_SUCCESS) {
				// Read data.
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &generic_u32_1);
				MAX11136_stack_error();
				if (max11136_status == MAX11136_SUCCESS) {
					spsws_ctx.measurements.vsrc_mv.data[spsws_ctx.measurements.vsrc_mv.count] = (uint16_t) generic_u32_1;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.vsrc_mv);
				}
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &generic_u32_1);
				MAX11136_stack_error();
				spsws_ctx.sigfox_monitoring_data.vcap_mv = (max11136_status == MAX11136_SUCCESS) ? (uint16_t) generic_u32_1 : SPSWS_ERROR_VALUE_ANALOG_12BITS;
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &generic_u32_1);
				MAX11136_stack_error();
				if (max11136_status == MAX11136_SUCCESS) {
					spsws_ctx.measurements.light_percent.data[spsws_ctx.measurements.light_percent.count] = (uint8_t) generic_u32_1;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.light_percent);
				}
			}
			else {
				// Set error value for VCAP.
				spsws_ctx.sigfox_monitoring_data.vcap_mv = SPSWS_ERROR_VALUE_ANALOG_12BITS;
			}
			IWDG_reload();
			// Internal temperature/humidity sensor.
			sht3x_status = SHT3X_perform_measurements(SHT3X_INT_I2C_ADDRESS);
			SHT3X_stack_error();
			// Check status.
			if (sht3x_status == SHT3X_SUCCESS) {
				// Read data.
				sht3x_status = SHT3X_get_temperature(&temperature);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					math_status = MATH_int32_to_signed_magnitude(temperature, 7, &generic_u32_1);
					MATH_stack_error();
					if (math_status == MATH_SUCCESS) {
						spsws_ctx.measurements.tpcb_degrees.data[spsws_ctx.measurements.tpcb_degrees.count] = (uint8_t) generic_u32_1;
						_SPSWS_increment_measurement_count(spsws_ctx.measurements.tpcb_degrees);
					}
				}
				sht3x_status = SHT3X_get_humidity(&generic_u8);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					spsws_ctx.measurements.hpcb_percent.data[spsws_ctx.measurements.hpcb_percent.count] = generic_u8;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.hpcb_percent);
				}
			}
			IWDG_reload();
			// External temperature/humidity sensor.
#ifdef HW1_0
			if (sht3x_status == SHT3X_SUCCESS) {
				spsws_ctx.measurements.tamb_degrees.data[spsws_ctx.measurements.tamb_degrees.count] = (uint8_t) generic_u32_1;
				_SPSWS_increment_measurement_count(spsws_ctx.measurements.tamb_degrees);
				spsws_ctx.measurements.hamb_percent.data[spsws_ctx.measurements.hamb_percent.count] = generic_u8;
				_SPSWS_increment_measurement_count(spsws_ctx.measurements.hamb_percent);
			}
#endif
#ifdef HW2_0
			sht3x_status = SHT3X_perform_measurements(SHT3X_EXT_I2C_ADDRESS);
			SHT3X_stack_error();
			// Check status.
			if (sht3x_status == SHT3X_SUCCESS) {
				sht3x_status = SHT3X_get_temperature(&temperature);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					math_status = MATH_int32_to_signed_magnitude(temperature, 7, &generic_u32_1);
					MATH_stack_error();
					if (math_status == MATH_SUCCESS) {
						spsws_ctx.measurements.tamb_degrees.data[spsws_ctx.measurements.tamb_degrees.count] = (uint8_t) generic_u32_1;
						_SPSWS_increment_measurement_count(spsws_ctx.measurements.tamb_degrees);
					}
				}
				sht3x_status = SHT3X_get_humidity(&generic_u8);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					spsws_ctx.measurements.hamb_percent.data[spsws_ctx.measurements.hamb_percent.count] = generic_u8;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.hamb_percent);
				}
			}
#endif
			IWDG_reload();
			// External pressure/temperature sensor.
			dps310_status = DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_stack_error();
			// Check status.
			if (dps310_status == DPS310_SUCCESS) {
				// Read data.
				dps310_status = DPS310_get_pressure(&generic_u32_1);
				DPS310_stack_error();
				if (dps310_status == DPS310_SUCCESS) {
					spsws_ctx.measurements.patm_abs_tenth_hpa.data[spsws_ctx.measurements.patm_abs_tenth_hpa.count] = (generic_u32_1 / 10);
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.patm_abs_tenth_hpa);
				}
			}
			IWDG_reload();
			// External UV index sensor.
			si1133_status = SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_SENSORS);
			POWER_stack_error();
			// Check status.
			if (si1133_status == SI1133_SUCCESS) {
				// Read data.
				si1133_status = SI1133_get_uv_index(&generic_u8);
				SI1133_stack_error();
				if (si1133_status == SI1133_SUCCESS) {
					spsws_ctx.measurements.uv_index.data[spsws_ctx.measurements.uv_index.count] = generic_u8;
					_SPSWS_increment_measurement_count(spsws_ctx.measurements.uv_index);
				}
			}
#ifdef SPSWS_WIND_MEASUREMENT
			IWDG_reload();
			// Retrieve wind measurements.
			wind_status = WIND_get_speed(&generic_u32_1, &generic_u32_2);
			WIND_stack_error();
			spsws_ctx.sigfox_weather_data.wind_speed_average_kmh = (wind_status == WIND_SUCCESS) ? (generic_u32_1 / 1000) : SPSWS_ERROR_VALUE_WIND;
			spsws_ctx.sigfox_weather_data.wind_speed_peak_kmh = (wind_status == WIND_SUCCESS) ? (generic_u32_2 / 1000) : SPSWS_ERROR_VALUE_WIND;
			wind_status = WIND_get_direction(&generic_u32_1);
			// Do not store undefined error (meaning that no wind has been detected so no direction can be computed).
			if ((wind_status != WIND_SUCCESS) && (wind_status != (WIND_ERROR_BASE_MATH + MATH_ERROR_UNDEFINED))) {
				WIND_stack_error();
			}
			spsws_ctx.sigfox_weather_data.wind_direction_average_two_degrees = (wind_status == WIND_SUCCESS) ? (generic_u32_1 / 2) : SPSWS_ERROR_VALUE_WIND;
#else
#ifdef SPSWS_RAIN_MEASUREMENT
			spsws_ctx.sigfox_weather_data.wind_speed_average_kmh = 0;
			spsws_ctx.sigfox_weather_data.wind_speed_peak_kmh = 0;
			spsws_ctx.sigfox_weather_data.wind_direction_average_two_degrees = 0;
#endif
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
			// Retrieve rain measurements.
			rain_status = RAIN_get_rainfall(&generic_u8);
			RAIN_stack_error();
			spsws_ctx.sigfox_weather_data.rain_mm = (rain_status == RAIN_SUCCESS) ? generic_u8 : SPSWS_ERROR_VALUE_RAIN;
#else
#ifdef SPSWS_WIND_MEASUREMENT
			spsws_ctx.sigfox_weather_data.rain_mm = 0;
#endif
#endif
#ifdef SPSWS_FLOOD_MEASUREMENT
			// Check if flood level has changed.
			spsws_ctx.state = (spsws_ctx.flags.flood_alarm != 0) ? SPSWS_STATE_FLOOD_ALARM : SPSWS_STATE_OFF;
#else
			spsws_ctx.state = SPSWS_STATE_OFF;
#endif
			break;
		case SPSWS_STATE_MONITORING:
			IWDG_reload();
			// Compute average data.
			_SPSWS_compute_final_measurements();
			_SPSWS_reset_measurements();
			// Read status byte.
			spsws_ctx.sigfox_monitoring_data.status = spsws_ctx.status.all;
			// Send uplink monitoring frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
			application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_monitoring_data.frame);
			application_message.ul_payload_size_bytes = SPSWS_SIGFOX_MONITORING_DATA_SIZE;
			_SPSWS_send_sigfox_message(&application_message);
			// Compute next state.
			spsws_ctx.state = SPSWS_STATE_WEATHER_DATA;
			break;
		case SPSWS_STATE_WEATHER_DATA:
			IWDG_reload();
			// Send uplink weather frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
			application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_weather_data.frame);
			application_message.ul_payload_size_bytes = SPSWS_SIGFOX_WEATHER_DATA_SIZE;
			_SPSWS_send_sigfox_message(&application_message);
			// Compute next state.
			if (spsws_ctx.status.daily_rtc_calibration == 0) {
				// Perform RTC calibration.
				spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
			}
			else {
				if ((spsws_ctx.status.daily_geoloc == 0) && (spsws_ctx.flags.is_afternoon != 0)) {
					// Perform device geolocation.
					spsws_ctx.state = SPSWS_STATE_GEOLOC;
				}
				else {
					// Enter stop mode.
					spsws_ctx.state = SPSWS_STATE_OFF;
				}
			}
			break;
#ifdef SPSWS_FLOOD_MEASUREMENT
		case SPSWS_STATE_FLOOD_ALARM:
			// Read level and rainfall.
			rain_status = RAIN_get_rainfall(&generic_u8);
			RAIN_stack_error();
			spsws_ctx.sigfox_flood_data.rain_mm = generic_u8;
			spsws_ctx.sigfox_flood_data.level = spsws_ctx.flood_level;
			// Send uplink flood alarm frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
			application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_flood_data.frame);
			application_message.ul_payload_size_bytes = SPSWS_SIGFOX_FLOOD_DATA_SIZE;
			_SPSWS_send_sigfox_message(&application_message);
			// Reset flags.
			spsws_ctx.flags.flood_alarm = 0;
			spsws_ctx.flood_level_change_count = 0;
			// Next state.
			spsws_ctx.state = SPSWS_STATE_OFF;
			break;
#endif
		case SPSWS_STATE_GEOLOC:
			IWDG_reload();
			// Get position from GPS.
			power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			neom8n_status = NEOM8N_get_position(&spsws_ctx.position, SPSWS_GEOLOC_TIMEOUT_SECONDS, SPSWS_GPS_ALTITUDE_STABILITY_FILTER, &spsws_ctx.geoloc_fix_duration_seconds);
			// Note: error is never stacked since it is indicated by the dedicated timeout frame.
			power_status = POWER_disable(POWER_DOMAIN_GPS);
			POWER_stack_error();
			// Update flag whatever the result.
			spsws_ctx.status.daily_geoloc = 1;
			// Build Sigfox frame.
			if (neom8n_status == NEOM8N_SUCCESS) {
				spsws_ctx.sigfox_geoloc_data.latitude_degrees = spsws_ctx.position.lat_degrees;
				spsws_ctx.sigfox_geoloc_data.latitude_minutes = spsws_ctx.position.lat_minutes;
				spsws_ctx.sigfox_geoloc_data.latitude_seconds = spsws_ctx.position.lat_seconds;
				spsws_ctx.sigfox_geoloc_data.latitude_north_flag = spsws_ctx.position.lat_north_flag;
				spsws_ctx.sigfox_geoloc_data.longitude_degrees = spsws_ctx.position.long_degrees;
				spsws_ctx.sigfox_geoloc_data.longitude_minutes = spsws_ctx.position.long_minutes;
				spsws_ctx.sigfox_geoloc_data.longitude_seconds = spsws_ctx.position.long_seconds;
				spsws_ctx.sigfox_geoloc_data.longitude_east_flag = spsws_ctx.position.long_east_flag;
				spsws_ctx.sigfox_geoloc_data.altitude_meters = spsws_ctx.position.altitude;
				spsws_ctx.sigfox_geoloc_data.gps_fix_duration = spsws_ctx.geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
				application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_geoloc_data.frame);
				application_message.ul_payload_size_bytes = SPSWS_SIGFOX_GEOLOC_DATA_SIZE;
			}
			else {
				spsws_ctx.sigfox_geoloc_timeout_data.error_code = (ERROR_BASE_NEOM8N + neom8n_status);
				spsws_ctx.sigfox_geoloc_timeout_data.fix_duration_seconds = spsws_ctx.geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
				application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_geoloc_timeout_data.frame);
				application_message.ul_payload_size_bytes = SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE;
				spsws_ctx.sigfox_geoloc_data.frame[0] = spsws_ctx.geoloc_fix_duration_seconds;
			}
			IWDG_reload();
			// Send uplink geolocation frame.
			_SPSWS_send_sigfox_message(&application_message);
			// Reset geoloc variables.
			spsws_ctx.geoloc_fix_duration_seconds = 0;
			// Send error stack frame.
			spsws_ctx.state = SPSWS_STATE_ERROR_STACK;
			break;
		case SPSWS_STATE_ERROR_STACK:
			// Check stack.
			if (ERROR_stack_is_empty() == 0) {
				// Read error stack.
				for (idx=0 ; idx<(SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE / 2) ; idx++) {
					error_code = ERROR_stack_read();
					spsws_ctx.sigfox_error_stack_data[(2 * idx) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
					spsws_ctx.sigfox_error_stack_data[(2 * idx) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
				}
				// Send frame.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
				application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_error_stack_data);
				application_message.ul_payload_size_bytes = SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE;
				_SPSWS_send_sigfox_message(&application_message);
				// Reset error stack.
				ERROR_stack_init();
			}
			// Enter sleep mode.
			spsws_ctx.state = SPSWS_STATE_OFF;
			break;
		case SPSWS_STATE_OFF:
			IWDG_reload();
			spsws_ctx.flags.por = 0;
			spsws_ctx.flags.wake_up = 0;
			spsws_ctx.flags.fixed_hour_alarm = 0;
			spsws_ctx.seconds_counter = 0;
#ifdef SPSWS_WIND_MEASUREMENT
			WIND_start_continuous_measure();
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
			RAIN_start_continuous_measure();
#endif
			// Enter sleep mode.
			spsws_ctx.state = SPSWS_STATE_SLEEP;
			break;
		case SPSWS_STATE_SLEEP:
			// Enter sleep mode.
			IWDG_reload();
			PWR_enter_stop_mode();
			IWDG_reload();
			// Check RTC alarm B flag (set every second).
			if (RTC_get_alarm_b_flag() != 0) {
#ifdef SPSWS_WIND_MEASUREMENT
				wind_status = WIND_tick_second();
				WIND_stack_error();
#endif
#ifdef SPSWS_FLOOD_MEASUREMENT
				// Check flood level if there is no pending alarm.
				if (spsws_ctx.flags.flood_alarm == 0) {
					rain_status = RAIN_get_flood_level(&generic_u8);
					RAIN_stack_error();
					if (spsws_ctx.flood_level != generic_u8) {
						spsws_ctx.flood_level_change_count++;
					}
					else {
						spsws_ctx.flood_level_change_count = 0;
					}
					// Check change count.
					if (spsws_ctx.flood_level_change_count >= SPSWS_FLOOD_LEVEL_CHANGE_THRESHOLD) {
						spsws_ctx.flood_level = generic_u8;
						spsws_ctx.flags.flood_alarm = 1;
					}
				}
#endif
				spsws_ctx.seconds_counter++;
				// Check measurements period.
				if (spsws_ctx.seconds_counter >= MEASUREMENT_PERIOD_SECONDS) {
					// Wake-up for single measurement.
					spsws_ctx.flags.wake_up = 1;
				}
				RTC_clear_alarm_b_flag();
			}
			// Check RTC alarm A flag (set every fixed hour).
			if (RTC_get_alarm_a_flag() != 0) {
				// Clear RTC flags and set local flag.
				spsws_ctx.flags.fixed_hour_alarm = 1;
				spsws_ctx.flags.wake_up = 1;
				RTC_clear_alarm_a_flag();
			}
			// Wake-up if required.
			if (spsws_ctx.flags.wake_up != 0) {
#ifdef SPSWS_WIND_MEASUREMENT
				WIND_stop_continuous_measure();
#endif
#ifdef SPSWS_RAIN_MEASUREMENT
				RAIN_stop_continuous_measure();
#endif
				// Wake-up.
				spsws_ctx.state = SPSWS_STATE_WAKEUP;
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
/*******************************************************************/
int main (void) {
	// Init board.
	_SPSWS_init_context();
	_SPSWS_init_hw();
	// Init AT command layer.
	AT_init();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		PWR_enter_sleep_mode();
		// Perform AT command parsing.
		AT_task();
		IWDG_reload();
	}
	return 0;
}
#endif
