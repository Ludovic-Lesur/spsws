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
#include "types.h"
// Applicative.
#include "at.h"
#include "error.h"
#include "mode.h"
#include "sigfox_api.h"
#include "version.h"

/*** SPSWS macros ***/

// Timeouts.
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS		180
#define SPSWS_GEOLOC_TIMEOUT_SECONDS				120
// Sigfox payload lengths.
#define SPSWS_SIGFOX_STARTUP_DATA_LENGTH			8
#define SPSWS_SIGFOX_ERROR_DATA_LENGTH				12
#ifdef IM
#define SPSWS_SIGFOX_WEATHER_DATA_LENGTH			6
#else
#define SPSWS_SIGFOX_WEATHER_DATA_LENGTH			10
#endif
#define SPSWS_SIGFOX_MONITORING_DATA_LENGTH			9
#define SPSWS_SIGFOX_GEOLOC_DATA_LENGTH				11
#define SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH		1
#define SPSWS_SIGFOX_ERROR_STACK_DATA_LENGTH		(ERROR_STACK_DEPTH * 2)
#ifdef FLOOD_DETECTION
#define SPSWS_SIGFOX_FLOOD_DATA_LENGTH				2
#define SPSWS_FLOOD_LEVEL_CHANGE_THRESHOLD			10 // Number of consecutive reading to validate a new flood level.
#endif
// Error values.
#define SPSWS_ERROR_VALUE_VOLTAGE_12BITS			0xFFF
#define SPSWS_ERROR_VALUE_VOLTAGE_16BITS			0xFFFF
#define SPSWS_ERROR_VALUE_LIGHT						0xFF
#define SPSWS_ERROR_VALUE_TEMPERATURE				0x7F
#define SPSWS_ERROR_VALUE_HUMIDITY					0xFF
#define SPSWS_ERROR_VALUE_UV_INDEX					0xFF
#define SPSWS_ERROR_VALUE_PRESSURE					0xFFFF
#define SPSWS_ERROR_VALUE_WIND						0xFF
// Measurements buffers length.
#define MEASUREMENT_PERIOD_SECONDS					60
#define MEASUREMENT_BUFFER_LENGTH					(3600 / MEASUREMENT_PERIOD_SECONDS)

/*** SPSWS structures ***/

typedef enum {
	SPSWS_STATE_STARTUP,
	SPSWS_STATE_WAKEUP,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
#ifdef FLOOD_DETECTION
	SPSWS_STATE_FLOOD_ALARM,
#endif
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_ERROR_STACK,
	SPSWS_STATE_OFF,
	SPSWS_STATE_SLEEP,
	SPSWS_STATE_LAST
} SPSWS_state_t;

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

typedef union {
	struct {
		unsigned por : 1;
		unsigned hour_changed : 1;
		unsigned day_changed : 1;
		unsigned is_afternoon : 1;
		unsigned geoloc_timeout : 1;
		unsigned wake_up : 1;
		unsigned fixed_hour_alarm : 1;
#ifdef FLOOD_DETECTION
		unsigned flood_alarm : 1;
#endif
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} SPSWS_flags_t;

typedef struct {
	uint8_t data[MEASUREMENT_BUFFER_LENGTH];
	uint8_t count;
	uint8_t full_flag;
} SPSWS_measurement_u8_t;

typedef struct {
	uint16_t data[MEASUREMENT_BUFFER_LENGTH];
	uint8_t count;
	uint8_t full_flag;
} SPSWS_measurement_u16_t;

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
	// Note: Vcap is not averaged since the last value is the most relevant.
} SPSWS_measurements_t;

// Sigfox start-up frame data format.
typedef union {
	uint8_t frame[SPSWS_SIGFOX_STARTUP_DATA_LENGTH];
	struct {
		unsigned reset_reason : 8;
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_startup_data_t;

// Sigfox weather frame data format.
typedef union {
	uint8_t frame[SPSWS_SIGFOX_WEATHER_DATA_LENGTH];
	struct {
		unsigned tamb_degrees : 8;
		unsigned hamb_percent : 8;
		unsigned light_percent : 8;
		unsigned uv_index : 8;
		unsigned patm_abs_tenth_hpa : 16;
#ifdef CM
		unsigned average_wind_speed_kmh : 8;
		unsigned peak_wind_speed_kmh : 8;
		unsigned average_wind_direction_two_degrees : 8;
		unsigned rain_mm : 8;
#endif
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_sigfox_weather_data_t;

// Sigfox monitoring frame data format.
typedef union {
	uint8_t frame[SPSWS_SIGFOX_MONITORING_DATA_LENGTH];
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

// Sigfox geolocation frame data format.
typedef union {
	uint8_t frame[SPSWS_SIGFOX_GEOLOC_DATA_LENGTH];
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

#ifdef FLOOD_DETECTION
typedef union {
	uint8_t frame[SPSWS_SIGFOX_FLOOD_DATA_LENGTH];
	struct {
		unsigned level : 8;
		unsigned rain_mm : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));;
} SPSWS_sigfox_flood_data_t;
#endif

typedef struct {
	// Global.
	SPSWS_state_t state;
	SPSWS_flags_t flags;
	// Clocks.
	uint32_t lsi_frequency_hz;
	uint8_t lse_running;
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
#ifdef FLOOD_DETECTION
	uint8_t flood_level;
	uint8_t flood_level_change_count;
	SPSWS_sigfox_flood_data_t sigfox_flood_data;
#endif
	// Geoloc.
	NEOM8N_position_t position;
	uint32_t geoloc_fix_duration_seconds;
	SPSWS_sigfox_geoloc_data_t sigfox_geoloc_data;
	// Error stack.
	ERROR_t error_stack[ERROR_STACK_DEPTH];
	uint8_t sigfox_error_stack_data[SPSWS_SIGFOX_ERROR_STACK_DATA_LENGTH];
	// Sigfox.
	sfx_rc_t sigfox_rc;
	sfx_u8 sigfox_downlink_data[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
} SPSWS_context_t;

/*** SPSWS global variables ***/

static SPSWS_context_t spsws_ctx;

/*** SPSWS local functions ***/

/* RESET ALL MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
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
#ifdef CM
	WIND_reset_data();
	RAIN_reset_data();
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

/* COMMON INIT FUNCTION FOR MAIN CONTEXT.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_init_context(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Init context.
	spsws_ctx.state = SPSWS_STATE_STARTUP;
	spsws_ctx.flags.all = 0;
	spsws_ctx.flags.por = 1;
	spsws_ctx.lsi_frequency_hz = 0;
	spsws_ctx.lse_running = 0;
	spsws_ctx.geoloc_fix_duration_seconds = 0;
	spsws_ctx.status.station_mode = SPSWS_MODE;
	spsws_ctx.seconds_counter = 0;
	_SPSWS_reset_measurements();
#ifdef FLOOD_DETECTION
	spsws_ctx.flood_level = 0;
	spsws_ctx.flood_level_change_count = 0;
#endif
	// Read status byte.
	nvm_status = NVM_read_byte(NVM_ADDRESS_STATUS, &spsws_ctx.status.all);
	NVM_error_check();
	// Reset all daily flags.
	spsws_ctx.status.first_rtc_calibration = 0;
	spsws_ctx.status.daily_rtc_calibration = 0;
	spsws_ctx.status.daily_geoloc = 0;
	spsws_ctx.status.daily_downlink = 0;
	// Set Sigfox RC.
	spsws_ctx.sigfox_rc = (sfx_rc_t) RC1;
}

/* CONFIGURE MCU CLOCK TREE FOR ACTIVE OPERATION.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_mcu_clock_wake_up(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	// Turn HSE on.
	rcc_status = RCC_switch_to_hse();
	RCC_error_check();
	// Check status.
	if (rcc_status == RCC_SUCCESS) {
		spsws_ctx.status.mcu_clock_source = 1;
	}
	else {
		spsws_ctx.status.mcu_clock_source = 0;
		rcc_status = RCC_switch_to_hsi();
		RCC_error_check();
	}
}

/* WAKE-UP RADIO TCXO.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_rf_clock_wake_up(void) {
	// Local variables.
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Turn RF TCXO on.
	sx1232_status = SX1232_tcxo(1);
	SX1232_error_check();
}

#if (defined IM || defined CM)
/* CONFIGURE CLOCK TREE FOR SLEEP OPERATION.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_clock_sleep(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Turn HSE off.
	rcc_status = RCC_switch_to_hsi();
	RCC_error_check();
	// Turn radio TCXO off.
	sx1232_status = SX1232_tcxo(0);
	SX1232_error_check();
}
#endif

/* COMMON INIT FUNCTION FOR PERIPHERALS AND COMPONENTS.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	uint8_t device_id_lsbyte = 0;
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	NVM_init();
	// Init GPIOs.
	GPIO_init(); // Required for clock tree configuration.
	EXTI_init(); // Required to clear RTC flags (EXTI 17).
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Reset RTC.
	RTC_reset();
	// Start oscillators.
	rcc_status = RCC_enable_lsi();
	RCC_error_check();
	spsws_ctx.status.lsi_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	rcc_status = RCC_enable_lse();
	RCC_error_check();
	spsws_ctx.status.lse_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	// High speed oscillator.
	IWDG_reload();
	_SPSWS_mcu_clock_wake_up();
	// Get LSI effective frequency (must be called after HSx initialization and before RTC initialization).
	rcc_status = RCC_get_lsi_frequency(&spsws_ctx.lsi_frequency_hz);
	RCC_error_check();
	if (rcc_status != RCC_SUCCESS) spsws_ctx.lsi_frequency_hz = RCC_LSI_FREQUENCY_HZ;
	IWDG_reload();
	// Read LS byte of the device ID to add a random delay in RTC alarm.
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_DEVICE_ID, &device_id_lsbyte);
	NVM_error_check();
	// RTC (only at POR).
	spsws_ctx.lse_running = spsws_ctx.status.lse_status;
	rtc_status = RTC_init(&spsws_ctx.lse_running, spsws_ctx.lsi_frequency_hz, device_id_lsbyte);
	RTC_error_check();
	// Update LSE status if RTC failed to start on it.
	if (spsws_ctx.lse_running == 0) {
		spsws_ctx.status.lse_status = 0;
	}
	IWDG_reload();
	// Internal.
	AES_init();
	DMA1_init_channel6();
	LPTIM1_init(spsws_ctx.lsi_frequency_hz);
	// Analog.
	adc1_status = ADC1_init();
	ADC1_error_check();
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
	DPS310_init();
	SX1232_init();
	SKY13317_init();
	NEOM8N_init();
	MAX11136_init();
#ifdef CM
	WIND_init();
	RAIN_init();
#endif
}

/* GENERIC FUNCTION TO MANAGE MEASUREMENT COUNTER.
 * @param measurement_struct:	Measurement structures (u8 or u16).
 * @return:						None.
 */
#define SPSWS_increment_measurement_count(measurement_struct) { \
	/* Increment index */ \
	measurement_struct.count++; \
	/* Manage rollover and flag */ \
	if (measurement_struct.count >= MEASUREMENT_BUFFER_LENGTH) { \
		measurement_struct.count = 0; \
		measurement_struct.full_flag = 1; \
	} \
}

#if (defined IM || defined CM)
/* CHECK IF HOUR OR DATE AS CHANGED SINCE PREVIOUS WAKE-UP AND UPDATE AFTERNOON FLAG.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_update_time_flags(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t nvm_byte = 0;
	uint8_t local_utc_offset = 0;
	int8_t local_hour = 0;
	// Retrieve current time from RTC.
	RTC_get_time(&spsws_ctx.current_time);
	// Retrieve previous wake-up time from NVM.
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), &nvm_byte);
	NVM_error_check();
	spsws_ctx.previous_wake_up_time.year = (nvm_byte << 8);
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), &nvm_byte);
	NVM_error_check();
	spsws_ctx.previous_wake_up_time.year |= nvm_byte;
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, &spsws_ctx.previous_wake_up_time.month);
	NVM_error_check();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, &spsws_ctx.previous_wake_up_time.date);
	NVM_error_check();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, &spsws_ctx.previous_wake_up_time.hours);
	NVM_error_check();
	// Check time are differents (avoiding false wake-up due to RTC recalibration).
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

#if (defined IM || defined CM)
/* UPDATE PREVIOUS WAKE-UP TIMESTAMP IN NVM.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_update_pwut(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Retrieve current time from RTC.
	RTC_get_time(&spsws_ctx.current_time);
	// Update previous wake-up time.
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), ((spsws_ctx.current_time.year & 0xFF00) >> 8));
	NVM_error_check();
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), ((spsws_ctx.current_time.year & 0x00FF) >> 0));
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, spsws_ctx.current_time.month);
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, spsws_ctx.current_time.date);
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, spsws_ctx.current_time.hours);
	NVM_error_check();
}
#endif

#if (defined IM || defined CM)
/* COMPUTE FINAL MEASUREMENTS FROM BUFFERS.
 * @param:	None.
 * @return:	None.
 */
static void _SPSWS_compute_final_measurements(void) {
	// Local variables.
	uint8_t data_length = 0;
	// Temperature
	data_length = (spsws_ctx.measurements.tamb_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tamb_degrees.count;
	spsws_ctx.sigfox_weather_data.tamb_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : MATH_min_u8(spsws_ctx.measurements.tamb_degrees.data, data_length);
	// Humidity.
	data_length = (spsws_ctx.measurements.hamb_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.hamb_percent.count;
	spsws_ctx.sigfox_weather_data.hamb_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_HUMIDITY : MATH_median_filter_u8(spsws_ctx.measurements.hamb_percent.data, data_length, 0);
	// Light.
	data_length = (spsws_ctx.measurements.light_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.light_percent.count;
	spsws_ctx.sigfox_weather_data.light_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_LIGHT : MATH_median_filter_u8(spsws_ctx.measurements.light_percent.data, data_length, 0);
	// UV index.
	data_length = (spsws_ctx.measurements.uv_index.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.uv_index.count;
	spsws_ctx.sigfox_weather_data.uv_index = (data_length == 0) ? SPSWS_ERROR_VALUE_UV_INDEX : MATH_max_u8(spsws_ctx.measurements.uv_index.data, data_length);
	// Absolute pressure.
	data_length = (spsws_ctx.measurements.patm_abs_tenth_hpa.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.patm_abs_tenth_hpa.count;
	spsws_ctx.sigfox_weather_data.patm_abs_tenth_hpa = (data_length == 0) ? SPSWS_ERROR_VALUE_PRESSURE : MATH_median_filter_u16(spsws_ctx.measurements.patm_abs_tenth_hpa.data, data_length, 0);
	// MCU temperature.
	data_length = (spsws_ctx.measurements.tmcu_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tmcu_degrees.count;
	spsws_ctx.sigfox_monitoring_data.tmcu_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : MATH_min_u8(spsws_ctx.measurements.tmcu_degrees.data, data_length);
	// PCB temperature.
	data_length = (spsws_ctx.measurements.tpcb_degrees.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.tpcb_degrees.count;
	spsws_ctx.sigfox_monitoring_data.tpcb_degrees = (data_length == 0) ? SPSWS_ERROR_VALUE_TEMPERATURE : MATH_min_u8(spsws_ctx.measurements.tpcb_degrees.data, data_length);
	// PCB humidity.
	data_length = (spsws_ctx.measurements.hpcb_percent.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.hpcb_percent.count;
	spsws_ctx.sigfox_monitoring_data.hpcb_percent = (data_length == 0) ? SPSWS_ERROR_VALUE_HUMIDITY : MATH_median_filter_u8(spsws_ctx.measurements.hpcb_percent.data, data_length, 0);
	// Solar cell voltage.
	data_length = (spsws_ctx.measurements.vsrc_mv.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.vsrc_mv.count;
	spsws_ctx.sigfox_monitoring_data.vsrc_mv = (data_length == 0) ? SPSWS_ERROR_VALUE_VOLTAGE_16BITS : MATH_median_filter_u16(spsws_ctx.measurements.vsrc_mv.data, data_length, 0);
	// MCU voltage.
	data_length = (spsws_ctx.measurements.vmcu_mv.full_flag != 0) ? MEASUREMENT_BUFFER_LENGTH : spsws_ctx.measurements.vmcu_mv.count;
	spsws_ctx.sigfox_monitoring_data.vmcu_mv = (data_length == 0) ? SPSWS_ERROR_VALUE_VOLTAGE_12BITS : MATH_median_filter_u16(spsws_ctx.measurements.vmcu_mv.data, data_length, 0);
}
#endif

/*** SPSWS main function ***/

#if (defined IM || defined CM)
/* MAIN FUNCTION FOR IM MODE.
 * @param: 	None.
 * @return: 0.
 */
int32_t main (void) {
	// Init board.
	_SPSWS_init_context();
	_SPSWS_init_hw();
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	uint8_t idx = 0;
	int8_t temperature = 0;
	uint8_t generic_data_u8 = 0;
	uint32_t generic_data_u32_1 = 0;
#ifdef CM
	uint32_t generic_data_u32_2 = 0;
	WIND_status_t wind_status = WIND_SUCCESS;
#endif
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.state) {
		case SPSWS_STATE_STARTUP:
			IWDG_reload();
			_SPSWS_rf_clock_wake_up();
			// Fill reset reason and software version.
			spsws_ctx.sigfox_startup_data.reset_reason = ((RCC -> CSR) >> 24) & 0xFF;
			spsws_ctx.sigfox_startup_data.major_version = GIT_MAJOR_VERSION;
			spsws_ctx.sigfox_startup_data.minor_version = GIT_MINOR_VERSION;
			spsws_ctx.sigfox_startup_data.commit_index = GIT_COMMIT_INDEX;
			spsws_ctx.sigfox_startup_data.commit_id = GIT_COMMIT_ID;
			spsws_ctx.sigfox_startup_data.dirty_flag = GIT_DIRTY_FLAG;
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23);
			// Send SW version frame.
			sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_startup_data.frame, SPSWS_SIGFOX_STARTUP_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
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
					// Fixed hour wake-up.
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
					// Switch to external clock.
					_SPSWS_mcu_clock_wake_up();
					_SPSWS_rf_clock_wake_up();
					// Next state.
					spsws_ctx.state = SPSWS_STATE_MEASURE;
				}
				else {
					// False detection due to RTC recalibration.
					spsws_ctx.state = SPSWS_STATE_OFF;
				}
			}
			else {
				// Intermediate measure wake-up.
				spsws_ctx.state = SPSWS_STATE_MEASURE;
#ifdef FLOOD_DETECTION
				// Turn clocks on if alarm has to be sent.
				if (spsws_ctx.flags.flood_alarm != 0) {
					SPSWS_mcu_clock_wake_up();
					SPSWS_rf_clock_wake_up();
				}
#endif
			}
			break;
		case SPSWS_STATE_RTC_CALIBRATION:
			IWDG_reload();
			// Get current time from GPS.
			lpuart1_status = LPUART1_power_on();
			LPUART1_error_check();
			neom8n_status = NEOM8N_get_time(&spsws_ctx.current_time, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS);
			NEOM8N_error_check();
			LPUART1_power_off();
			// Calibrate RTC if time is available.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Update RTC registers.
				rtc_status = RTC_calibrate(&spsws_ctx.current_time);
				RTC_error_check();
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
			adc1_status = ADC1_perform_measurements();
			ADC1_error_check();
			// Check status.
			if (adc1_status == ADC_SUCCESS) {
				// Read data.
				ADC1_get_tmcu(&temperature);
				math_status = MATH_one_complement(temperature, 7, &generic_data_u32_1);
				MATH_error_check();
				spsws_ctx.measurements.tmcu_degrees.data[spsws_ctx.measurements.tmcu_degrees.count] = (uint8_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.tmcu_degrees);
				adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &generic_data_u32_1);
				ADC1_error_check();
				spsws_ctx.measurements.vmcu_mv.data[spsws_ctx.measurements.vmcu_mv.count] = generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.vmcu_mv);
			}
			// Retrieve external ADC data.
			i2c1_status = I2C1_power_on(); // Must be called before ADC since LDR is on the sensors module (powered by I2C supply).
			I2C1_error_check();
#ifdef HW1_0
			spi_status = SPI1_power_on();
			SPI1_error_check();
#endif
#ifdef HW2_0
			spi_status = SPI2_power_on();
			SPI2_error_check();
#endif
			IWDG_reload();
			max11136_status = MAX11136_perform_measurements();
			MAX11136_error_check();
#ifdef HW1_0
			SPI1_power_off();
#endif
#ifdef HW2_0
			SPI2_power_off();
#endif
			// Check status.
			if (max11136_status == MAX11136_SUCCESS) {
				// Read data.
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &generic_data_u32_1);
				MAX11136_error_check();
				spsws_ctx.measurements.vsrc_mv.data[spsws_ctx.measurements.vsrc_mv.count] = (uint16_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.vsrc_mv);
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &generic_data_u32_1);
				MAX11136_error_check();
				spsws_ctx.sigfox_monitoring_data.vcap_mv = (uint16_t) generic_data_u32_1;
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &generic_data_u32_1);
				MAX11136_error_check();
				spsws_ctx.measurements.light_percent.data[spsws_ctx.measurements.light_percent.count] = (uint8_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.light_percent);
			}
			else {
				// Set error value for Vcap.
				spsws_ctx.sigfox_monitoring_data.vcap_mv = SPSWS_ERROR_VALUE_VOLTAGE_12BITS;
			}
			// Internal temperature/humidity sensor.
#ifdef HW1_0
			// Must be called again because SPI1 power function turned off the sensors.
			i2c1_status = I2C1_power_on();
			I2C1_error_check();
#endif
			IWDG_reload();
			sht3x_status = SHT3X_perform_measurements(SHT3X_INT_I2C_ADDRESS);
			SHT3X_INT_error_check();
			// Check status.
			if (sht3x_status == SHT3X_SUCCESS) {
				// Read data.
				SHT3X_get_temperature(&temperature);
				math_status = MATH_one_complement(temperature, 7, &generic_data_u32_1);
				MATH_error_check();
				spsws_ctx.measurements.tpcb_degrees.data[spsws_ctx.measurements.tpcb_degrees.count] = (uint8_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.tpcb_degrees);
				SHT3X_get_humidity(&generic_data_u8);
				spsws_ctx.measurements.hpcb_percent.data[spsws_ctx.measurements.hpcb_percent.count] = generic_data_u8;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.hpcb_percent);
			}
			// External temperature/humidity sensor.
#ifdef HW1_0
			if (sht3x_status == SHT3X_SUCCESS) {
				spsws_ctx.measurements.tamb_degrees.data[spsws_ctx.measurements.tamb_degrees.count] = (uint8_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.tamb_degrees);
				spsws_ctx.measurements.hamb_percent.data[spsws_ctx.measurements.hamb_percent.count] = generic_data_u8;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.hamb_percent);
			}
#endif
#ifdef HW2_0
			IWDG_reload();
			sht3x_status = SHT3X_perform_measurements(SHT3X_EXT_I2C_ADDRESS);
			SHT3X_EXT_error_check();
			// Check status.
			if (sht3x_status == SHT3X_SUCCESS) {
				SHT3X_get_temperature(&temperature);
				MATH_one_complement(temperature, 7, &generic_data_u32_1);
				spsws_ctx.measurements.tamb_degrees.data[spsws_ctx.measurements.tamb_degrees.count] = (uint8_t) generic_data_u32_1;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.tamb_degrees);
				SHT3X_get_humidity(&generic_data_u8);
				spsws_ctx.measurements.hamb_percent.data[spsws_ctx.measurements.hamb_percent.count] = generic_data_u8;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.hamb_percent);
			}
#endif
			// External pressure/temperature sensor.
			IWDG_reload();
			dps310_status = DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
			DPS310_error_check();
			// Check status.
			if (dps310_status == DPS310_SUCCESS) {
				// Read data.
				DPS310_get_pressure(&generic_data_u32_1);
				spsws_ctx.measurements.patm_abs_tenth_hpa.data[spsws_ctx.measurements.patm_abs_tenth_hpa.count] = (generic_data_u32_1 / 10);
				SPSWS_increment_measurement_count(spsws_ctx.measurements.patm_abs_tenth_hpa);
			}
			// External UV index sensor.
			IWDG_reload();
			si1133_status = SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
			SI1133_error_check();
			// Turn sensors off.
			I2C1_power_off();
			// Check status.
			if (si1133_status == SI1133_SUCCESS) {
				// Read data.
				SI1133_get_uv_index(&generic_data_u8);
				spsws_ctx.measurements.uv_index.data[spsws_ctx.measurements.uv_index.count] = generic_data_u8;
				SPSWS_increment_measurement_count(spsws_ctx.measurements.uv_index);
			}
#ifdef CM
			IWDG_reload();
			// Retrieve wind measurements.
			WIND_get_speed(&generic_data_u32_1, &generic_data_u32_2);
			spsws_ctx.sigfox_weather_data.average_wind_speed_kmh = (generic_data_u32_1 / 1000);
			spsws_ctx.sigfox_weather_data.peak_wind_speed_kmh = (generic_data_u32_2 / 1000);
			wind_status = WIND_get_direction(&generic_data_u32_1);
			// Do not store undefined error (meaning that no wind has been detected so no direction can be computed).
			if ((wind_status != WIND_SUCCESS) && (wind_status != (WIND_ERROR_BASE_MATH + MATH_ERROR_UNDEFINED))) {
				WIND_error_check();
			}
			spsws_ctx.sigfox_weather_data.average_wind_direction_two_degrees = (wind_status == WIND_SUCCESS) ? (generic_data_u32_1 / 2) : SPSWS_ERROR_VALUE_WIND;
			// Retrieve rain measurements.
			RAIN_get_pluviometry(&generic_data_u8);
			spsws_ctx.sigfox_weather_data.rain_mm = generic_data_u8;
#endif
			// Read status byte.
			spsws_ctx.sigfox_monitoring_data.status = spsws_ctx.status.all;
			// Check alarm flag.
			if (spsws_ctx.flags.fixed_hour_alarm != 0) {
				// Compute average data.
				_SPSWS_compute_final_measurements();
				_SPSWS_reset_measurements();
				// Send data.
				spsws_ctx.state = SPSWS_STATE_MONITORING;
			}
			else {
#ifdef FLOOD_DETECTION
				// Check if flood level has changed.
				spsws_ctx.state = (spsws_ctx.flags.flood_alarm != 0) ? SPSWS_STATE_FLOOD_ALARM : SPSWS_STATE_OFF;
#else
				spsws_ctx.state = SPSWS_STATE_OFF;
#endif
			}
			break;
		case SPSWS_STATE_MONITORING:
			IWDG_reload();
			// Send uplink monitoring frame.
			sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_monitoring_data.frame, SPSWS_SIGFOX_MONITORING_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
			// Compute next state.
			spsws_ctx.state = SPSWS_STATE_WEATHER_DATA;
			break;
		case SPSWS_STATE_WEATHER_DATA:
			IWDG_reload();
			// Send uplink weather frame.
			sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_weather_data.frame, SPSWS_SIGFOX_WEATHER_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
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
#ifdef FLOOD_DETECTION
		case SPSWS_STATE_FLOOD_ALARM:
			// Read level and pluviometry.
			RAIN_get_pluviometry(&generic_data_u8);
			spsws_ctx.sigfox_flood_data.rain_mm = generic_data_u8;
			spsws_ctx.sigfox_flood_data.level = spsws_ctx.flood_level;
			// Send uplink flood alarm frame.
			sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_flood_data.frame, SPSWS_SIGFOX_FLOOD_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
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
			lpuart1_status = LPUART1_power_on();
			LPUART1_error_check();
			neom8n_status = NEOM8N_get_position(&spsws_ctx.position, SPSWS_GEOLOC_TIMEOUT_SECONDS, &spsws_ctx.geoloc_fix_duration_seconds);
			NEOM8N_error_check();
			LPUART1_power_off();
			// Update flag whatever the result.
			spsws_ctx.status.daily_geoloc = 1;
			// Parse result.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Build frame.
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
			}
			else {
				spsws_ctx.sigfox_geoloc_data.frame[0] = spsws_ctx.geoloc_fix_duration_seconds;
				spsws_ctx.flags.geoloc_timeout = 1;
			}
			IWDG_reload();
			// Send uplink geolocation frame.
			sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_geoloc_data.frame, ((spsws_ctx.flags.geoloc_timeout) ? SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH : SPSWS_SIGFOX_GEOLOC_DATA_LENGTH), spsws_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
			// Reset geoloc variables.
			spsws_ctx.flags.geoloc_timeout = 0;
			spsws_ctx.geoloc_fix_duration_seconds = 0;
			// Send error stack frame.
			spsws_ctx.state = SPSWS_STATE_ERROR_STACK;
			break;
		case SPSWS_STATE_ERROR_STACK:
			// Check stack.
			if (ERROR_stack_is_empty() == 0) {
				// Read error stack.
				ERROR_stack_read(spsws_ctx.error_stack);
				// Convert to 8-bits little-endian array.
				for (idx=0 ; idx<SPSWS_SIGFOX_ERROR_STACK_DATA_LENGTH ; idx++) {
					spsws_ctx.sigfox_error_stack_data[idx] = spsws_ctx.error_stack[idx / 2] >> (8 * ((idx + 1) % 2));
				}
				// Send frame.
				sigfox_api_status = SIGFOX_API_open(&spsws_ctx.sigfox_rc);
				SIGFOX_API_error_check();
				if (sigfox_api_status == SFX_ERR_NONE) {
					sigfox_api_status = SIGFOX_API_send_frame(spsws_ctx.sigfox_error_stack_data, SPSWS_SIGFOX_ERROR_STACK_DATA_LENGTH, spsws_ctx.sigfox_downlink_data, 2, 0);
					SIGFOX_API_error_check();
				}
				sigfox_api_status = SIGFOX_API_close();
				SIGFOX_API_error_check();
				// Reset error stack.
				ERROR_stack_init();
			}
			// Enter sleep mode.
			spsws_ctx.state = SPSWS_STATE_OFF;
			break;
		case SPSWS_STATE_OFF:
			IWDG_reload();
			// Disable HSE.
			_SPSWS_clock_sleep();
			// Clear flags.
			spsws_ctx.flags.por = 0;
			spsws_ctx.flags.wake_up = 0;
			spsws_ctx.flags.fixed_hour_alarm = 0;
			spsws_ctx.seconds_counter = 0;
			// Store status byte in NVM.
			nvm_status = NVM_write_byte(NVM_ADDRESS_STATUS, spsws_ctx.status.all);
			NVM_error_check();
#ifdef CM
			// Re-start continuous measurements.
			WIND_start_continuous_measure();
			RAIN_start_continuous_measure();
			NVIC_enable_interrupt(NVIC_IT_EXTI_4_15);
#endif
			// Enable RTC interrupt.
			RTC_enable_alarm_b_interrupt();
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
#ifdef CM
				wind_status = WIND_measurement_period_callback();
				WIND_error_check();
#endif
#ifdef FLOOD_DETECTION
				// Check flood level if there is no pending alarm.
				if (spsws_ctx.flags.flood_alarm == 0) {
					RAIN_get_flood_level(&generic_data_u8);
					if (spsws_ctx.flood_level != generic_data_u8) {
						spsws_ctx.flood_level_change_count++;
					}
					else {
						spsws_ctx.flood_level_change_count = 0;
					}
					// Check change count.
					if (spsws_ctx.flood_level_change_count >= SPSWS_FLOOD_LEVEL_CHANGE_THRESHOLD) {
						spsws_ctx.flood_level = generic_data_u8;
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
				// Disable RTC alarm interrupts.
				RTC_disable_alarm_b_interrupt();
#ifdef CM
				// Stop continuous measurements.
				WIND_stop_continuous_measure();
				RAIN_stop_continuous_measure();
				NVIC_disable_interrupt(NVIC_IT_EXTI_4_15);
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
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int32_t main (void) {
	// Init board.
	_SPSWS_init_context();
	_SPSWS_init_hw();
	// Turn RF TCXO on.
	_SPSWS_rf_clock_wake_up();
	// Enable alarm interrupt to wake-up every seconds to clear watchdog.
	RTC_clear_alarm_b_flag();
	RTC_enable_alarm_b_interrupt();
	// Init AT command layer.
	AT_init();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		PWR_enter_sleep_mode();
		// Perform AT command parsing.
		AT_task();
		// Clear flag and watchdog.
		RTC_clear_alarm_b_flag();
		IWDG_reload();
	}
	return 0;
}
#endif
