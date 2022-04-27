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
// Error values.
#define SPSWS_ERROR_VALUE_VOLTAGE_12BITS			0xFFF
#define SPSWS_ERROR_VALUE_VOLTAGE_16BITS			0xFFFF
#define SPSWS_ERROR_VALUE_LIGHT						0xFF
#define SPSWS_ERROR_VALUE_TEMPERATURE				0x7F
#define SPSWS_ERROR_VALUE_HUMIDITY					0xFF
#define SPSWS_ERROR_VALUE_UV_INDEX					0xFF
#define SPSWS_ERROR_VALUE_PRESSURE					0xFFFF
#define SPSWS_ERROR_VALUE_WIND						0xFF

/*** SPSWS structures ***/

typedef enum {
	SPSWS_STATE_WAKE_UP,
	SPSWS_STATE_STARTUP,
	SPSWS_STATE_MEASURE,
	SPSWS_STATE_MONITORING,
	SPSWS_STATE_WEATHER_DATA,
	SPSWS_STATE_GEOLOC,
	SPSWS_STATE_RTC_CALIBRATION,
	SPSWS_STATE_ERROR_STACK,
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

// Sigfox start-up frame data format.
typedef union {
	unsigned char frame[SPSWS_SIGFOX_STARTUP_DATA_LENGTH];
	struct {
		unsigned reset_reason : 8;
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} SPSWS_sigfox_startup_data_t;

// Sigfox weather frame data format.
typedef union {
	unsigned char frame[SPSWS_SIGFOX_WEATHER_DATA_LENGTH];
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
	unsigned char frame[SPSWS_SIGFOX_MONITORING_DATA_LENGTH];
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
	unsigned char frame[SPSWS_SIGFOX_GEOLOC_DATA_LENGTH];
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
	SPSWS_sigfox_startup_data_t sigfox_startup_data;
	// Monitoring.
	SPSWS_status_t status;
	SPSWS_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Weather data.
	SPSWS_sigfox_weather_data_t sigfox_weather_data;
	// Geoloc.
	NEOM8N_position_t position;
	unsigned int geoloc_fix_duration_seconds;
	SPSWS_sigfox_geoloc_data_t sigfox_geoloc_data;
	// Error stack.
	ERROR_t error_stack[ERROR_STACK_DEPTH];
	unsigned char sigfox_error_stack_data[SPSWS_SIGFOX_ERROR_STACK_DATA_LENGTH];
	// Sigfox.
	sfx_rc_t sigfox_rc;
	sfx_u8 sigfox_downlink_data[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
} SPSWS_context_t;

/*** SPSWS global variables ***/

static SPSWS_context_t spsws_ctx;

/*** SPSWS local functions ***/

/* CONFIGURE CLOCK TREE FOR ACTIVE OPERATION.
 * @param:	None.
 * @return:	None.
 */
static void SPSWS_clock_wake_up(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	// Turn HSE on.
	rcc_status = RCC_switch_to_hse();
	RCC_error_check();
	// Check status.
	if (rcc_status == RCC_SUCCESS) {
		spsws_ctx.status.field.mcu_clock_source = 1;
	}
	else {
		spsws_ctx.status.field.mcu_clock_source = 0;
		rcc_status = RCC_switch_to_hsi();
		RCC_error_check();
	}
}

#if (defined IM || defined CM)
/* CONFIGURE CLOCK TREE FOR SLEEP OPERATION.
 * @param:	None.
 * @return:	None.
 */
static void SPSWS_clock_sleep(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	// Turn HSE off.
	rcc_status = RCC_switch_to_hsi();
	RCC_error_check();
}
#endif

/* COMMON INIT FUNCTION FOR PERIPHERALS AND COMPONENTS.
 * @param:	None.
 * @return:	None.
 */
static void SPSWS_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	unsigned char device_id_lsbyte = 0;
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
	spsws_ctx.status.field.lsi_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	rcc_status = RCC_enable_lse();
	RCC_error_check();
	spsws_ctx.status.field.lse_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	// Start independant watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	// High speed oscillator.
	IWDG_reload();
	SPSWS_clock_wake_up();
	// Get LSI effective frequency (must be called after HSx initialization and before RTC inititialization).
	rcc_status = RCC_get_lsi_frequency(&spsws_ctx.lsi_frequency_hz);
	RCC_error_check();
	if (rcc_status != RCC_SUCCESS) spsws_ctx.lsi_frequency_hz = RCC_LSI_FREQUENCY_HZ;
	IWDG_reload();
	// Read LS byte of the device ID to add a random delay in RTC alarm.
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_DEVICE_ID, &device_id_lsbyte);
	NVM_error_check();
	// RTC (only at POR).
	spsws_ctx.lse_running = spsws_ctx.status.field.lse_status;
	rtc_status = RTC_init(&spsws_ctx.lse_running, spsws_ctx.lsi_frequency_hz, device_id_lsbyte);
	RTC_error_check();
	// Update LSE status if RTC failed to start on it.
	if (spsws_ctx.lse_running == 0) {
		spsws_ctx.status.field.lse_status = 0;
	}
	IWDG_reload();
	// Internal.
	AES_init();
	DMA1_init_channel6();
	LPTIM1_init(spsws_ctx.lsi_frequency_hz);
	// Analog.
	adc_status = ADC1_init();
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
	SX1232_init();
	SKY13317_init();
	NEOM8N_init();
	MAX11136_init();
#ifdef CM
	WIND_init();
	RAIN_init();
#endif
}

/* COMMON INIT FUNCTION FOR MAIN CONTEXT.
 * @param:	None.
 * @return:	None.
 */
static void SPSWS_init_context(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Init context.
	spsws_ctx.state = SPSWS_STATE_WAKE_UP;
	spsws_ctx.flags.all = 0;
	spsws_ctx.flags.por = 1;
	spsws_ctx.lsi_frequency_hz = 0;
	spsws_ctx.lse_running = 0;
	spsws_ctx.geoloc_fix_duration_seconds = 0;
	spsws_ctx.status.field.station_mode = SPSWS_MODE;
	// Read status byte.
	nvm_status = NVM_read_byte(NVM_ADDRESS_STATUS, &spsws_ctx.status.raw_byte);
	NVM_error_check();
	// Reset all daily flags.
	spsws_ctx.status.field.first_rtc_calibration = 0;
	spsws_ctx.status.field.daily_rtc_calibration = 0;
	spsws_ctx.status.field.daily_geoloc = 0;
	spsws_ctx.status.field.daily_downlink = 0;
	// Set Sigfox RC.
	spsws_ctx.sigfox_rc = (sfx_rc_t) RC1;
}

#if (defined IM || defined CM)
/* CHECK IF HOUR OR DATE AS CHANGED SINCE PREVIOUS WAKE-UP AND UPDATE AFTERNOON FLAG.
 * @param:	None.
 * @return:	None.
 */
static void SPSWS_update_time_flags(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char nvm_byte = 0;
	unsigned char local_utc_offset = 0;
	signed char local_hour = 0;
	// Retrieve current timestamp from RTC.
	RTC_get_timestamp(&spsws_ctx.current_timestamp);
	// Retrieve previous wake-up timestamp from NVM.
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), &nvm_byte);
	NVM_error_check();
	spsws_ctx.previous_wake_up_timestamp.year = (nvm_byte << 8);
	nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), &nvm_byte);
	NVM_error_check();
	spsws_ctx.previous_wake_up_timestamp.year |= nvm_byte;
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, &spsws_ctx.previous_wake_up_timestamp.month);
	NVM_error_check();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, &spsws_ctx.previous_wake_up_timestamp.date);
	NVM_error_check();
	nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, &spsws_ctx.previous_wake_up_timestamp.hours);
	NVM_error_check();
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
	local_utc_offset = RTC_LOCAL_UTC_OFFSET_WINTER;
	if ((spsws_ctx.current_timestamp.month > RTC_WINTER_TIME_LAST_MONTH) && (spsws_ctx.current_timestamp.month < RTC_WINTER_TIME_FIRST_MONTH)) {
		local_utc_offset = RTC_LOCAL_UTC_OFFSET_SUMMER;
	}
	local_hour = (spsws_ctx.current_timestamp.hours + local_utc_offset) % RTC_NUMBER_OF_HOURS_PER_DAY;
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
static void SPSWS_update_pwut(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Retrieve current timestamp from RTC.
	RTC_get_timestamp(&spsws_ctx.current_timestamp);
	// Update previous wake-up timestamp.
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), ((spsws_ctx.current_timestamp.year & 0xFF00) >> 8));
	NVM_error_check();
	nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), ((spsws_ctx.current_timestamp.year & 0x00FF) >> 0));
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, spsws_ctx.current_timestamp.month);
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, spsws_ctx.current_timestamp.date);
	NVM_error_check();
	nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, spsws_ctx.current_timestamp.hours);
	NVM_error_check();
}
#endif

/*** SPSWS main function ***/

#if (defined IM || defined CM)
/* MAIN FUNCTION FOR IM MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init board.
	SPSWS_init_context();
	SPSWS_init_hw();
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	unsigned char idx = 0;
	signed char temperature = 0;
	unsigned char generic_data_u8 = 0;
	unsigned int generic_data_u32_1 = 0;
#ifdef CM
	unsigned int generic_data_u32_2 = 0;
	WIND_status_t wind_status = WIND_SUCCESS;
#endif
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (spsws_ctx.state) {
		case SPSWS_STATE_WAKE_UP:
			IWDG_reload();
			// Update flags.
			SPSWS_update_time_flags();
			// Check flag.
			if ((spsws_ctx.flags.hour_changed == 0) && (spsws_ctx.flags.por == 0)) {
				// False detection due to RTC recalibration.
				spsws_ctx.state = SPSWS_STATE_OFF;
			}
			else {
				// Valid wake-up.
				SPSWS_clock_wake_up();
				// Turn RF TCXO on.
				sx1232_status = SX1232_tcxo(1);
				SX1232_error_check();
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
				spsws_ctx.flags.hour_changed = 0; // Reset flag.
				// Next state.
				spsws_ctx.state = (spsws_ctx.flags.por != 0) ? SPSWS_STATE_STARTUP : SPSWS_STATE_MEASURE;
			}
			break;
		case SPSWS_STATE_STARTUP:
			IWDG_reload();
			// Fill reset reason and software version.
			spsws_ctx.sigfox_startup_data.field.reset_reason = ((RCC -> CSR) >> 24) & 0x0F;
			spsws_ctx.sigfox_startup_data.field.major_version = GIT_MAJOR_VERSION;
			spsws_ctx.sigfox_startup_data.field.minor_version = GIT_MINOR_VERSION;
			spsws_ctx.sigfox_startup_data.field.commit_index = GIT_COMMIT_INDEX;
			spsws_ctx.sigfox_startup_data.field.commit_id = GIT_COMMIT_ID;
			spsws_ctx.sigfox_startup_data.field.dirty_flag = GIT_DIRTY_FLAG;
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
		case SPSWS_STATE_RTC_CALIBRATION:
			IWDG_reload();
			// Get current timestamp from GPS.
			lpuart_status = LPUART1_power_on();
			LPUART1_error_check();
			neom8n_status = NEOM8N_get_time(&spsws_ctx.current_timestamp, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS);
			NEOM8N_error_check();
			LPUART1_power_off();
			// Calibrate RTC if timestamp is available.
			if (neom8n_status == NEOM8N_SUCCESS) {
				// Update RTC registers.
				rtc_status = RTC_calibrate(&spsws_ctx.current_timestamp);
				RTC_error_check();
				// Update PWUT when first calibration.
				if (spsws_ctx.status.field.first_rtc_calibration == 0) {
					SPSWS_update_pwut();
				}
				// Update calibration flags.
				spsws_ctx.status.field.first_rtc_calibration = 1;
				spsws_ctx.status.field.daily_rtc_calibration = 1;
			}
			// Send error stack at start
			spsws_ctx.state = (spsws_ctx.flags.por != 0) ? SPSWS_STATE_ERROR_STACK : SPSWS_STATE_OFF;
			break;
		case SPSWS_STATE_MEASURE:
			IWDG_reload();
			// Retrieve internal ADC data.
			adc_status = ADC1_perform_measurements();
			ADC1_error_check();
			// Check status.
			if (adc_status == ADC_SUCCESS) {
				// Read data.
				ADC1_get_tmcu(&temperature);
				math_status = MATH_one_complement(temperature, 7, &generic_data_u32_1);
				MATH_error_check();
				spsws_ctx.sigfox_monitoring_data.field.tmcu_degrees = (unsigned char) generic_data_u32_1;
				adc_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &generic_data_u32_1);
				ADC1_error_check();
				spsws_ctx.sigfox_monitoring_data.field.vmcu_mv = generic_data_u32_1;
			}
			else {
				// Set error values.
				spsws_ctx.sigfox_monitoring_data.field.tmcu_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
				spsws_ctx.sigfox_monitoring_data.field.vmcu_mv = SPSWS_ERROR_VALUE_VOLTAGE_12BITS;
			}
			// Retrieve external ADC data.
			i2c_status = I2C1_power_on(); // Must be called before ADC since LDR is on the sensors module (powered by I2C supply).
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
				spsws_ctx.sigfox_monitoring_data.field.vsrc_mv = (unsigned short) generic_data_u32_1;
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &generic_data_u32_1);
				MAX11136_error_check();
				spsws_ctx.sigfox_monitoring_data.field.vcap_mv = (unsigned short) generic_data_u32_1;
				max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &generic_data_u32_1);
				MAX11136_error_check();
				spsws_ctx.sigfox_weather_data.field.light_percent = (unsigned char) generic_data_u32_1;
			}
			else {
				// Set error values.
				spsws_ctx.sigfox_monitoring_data.field.vsrc_mv = SPSWS_ERROR_VALUE_VOLTAGE_16BITS;
				spsws_ctx.sigfox_monitoring_data.field.vcap_mv = SPSWS_ERROR_VALUE_VOLTAGE_12BITS;
				spsws_ctx.sigfox_weather_data.field.light_percent = SPSWS_ERROR_VALUE_LIGHT;
			}
			// Internal temperature/humidity sensor.
#ifdef HW1_0
			// Must be called again because SPI1 power function turned off the sensors.
			i2c_status = I2C1_power_on();
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
				spsws_ctx.sigfox_monitoring_data.field.tpcb_degrees = (unsigned char) generic_data_u32_1;
				SHT3X_get_humidity(&generic_data_u8);
				spsws_ctx.sigfox_monitoring_data.field.hpcb_percent = generic_data_u8;
			}
			else {
				// Set error values.
				spsws_ctx.sigfox_monitoring_data.field.tpcb_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
				spsws_ctx.sigfox_monitoring_data.field.hpcb_percent = SPSWS_ERROR_VALUE_HUMIDITY;
			}
			// External temperature/humidity sensor.
#ifdef HW1_0
			spsws_ctx.sigfox_weather_data.field.temperature_degrees = spsws_ctx.sigfox_monitoring_data.field.tpcb_degrees;
			spsws_ctx.sigfox_weather_data.field.humidity_percent = spsws_ctx.sigfox_monitoring_data.field.hpcb_percent;
#endif
#ifdef HW2_0
			IWDG_reload();
			sht3x_status = SHT3X_perform_measurements(SHT3X_EXT_I2C_ADDRESS);
			SHT3X_EXT_error_check();
			// Check status.
			if (sht3x_status == SHT3X_SUCCESS) {
				SHT3X_get_temperature(&temperature);
				MATH_one_complement(temperature, 7, &generic_data_u32_1);
				spsws_ctx.sigfox_weather_data.field.temperature_degrees = (unsigned char) generic_data_u32_1;
				SHT3X_get_humidity(&generic_data_u8);
				spsws_ctx.sigfox_weather_data.field.humidity_percent = generic_data_u8;
			}
			else {
				// Set error values.
				spsws_ctx.sigfox_weather_data.field.temperature_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
				spsws_ctx.sigfox_weather_data.field.humidity_percent = SPSWS_ERROR_VALUE_HUMIDITY;
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
				spsws_ctx.sigfox_weather_data.field.absolute_pressure_tenth_hpa = (generic_data_u32_1 / 10);
			}
			else {
				// Set error value.
				spsws_ctx.sigfox_weather_data.field.absolute_pressure_tenth_hpa = SPSWS_ERROR_VALUE_PRESSURE;
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
				spsws_ctx.sigfox_weather_data.field.uv_index = generic_data_u8;
			}
			else {
				// Set error value.
				spsws_ctx.sigfox_weather_data.field.uv_index = SPSWS_ERROR_VALUE_UV_INDEX;
			}
#ifdef CM
			IWDG_reload();
			// Retrieve wind measurements.
			WIND_get_speed(&generic_data_u32_1, &generic_data_u32_2);
			spsws_ctx.sigfox_weather_data.field.average_wind_speed_kmh = (generic_data_u32_1 / 1000);
			spsws_ctx.sigfox_weather_data.field.peak_wind_speed_kmh = (generic_data_u32_2 / 1000);
			wind_status = WIND_get_direction(&generic_data_u32_1);
			WIND_error_check();
			// Check status.
			spsws_ctx.sigfox_weather_data.field.average_wind_direction_two_degrees = (wind_status == WIND_SUCCESS) ? (generic_data_u32_1 / 2) : SPSWS_ERROR_VALUE_WIND;
			// Retrieve rain measurements.
			RAIN_get_pluviometry(&generic_data_u8);
			spsws_ctx.sigfox_weather_data.field.rain_mm = generic_data_u8;
#endif
			// Read status byte.
			spsws_ctx.sigfox_monitoring_data.field.status = spsws_ctx.status.raw_byte;
			// Compute next state.
			spsws_ctx.state = SPSWS_STATE_MONITORING;
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
		case SPSWS_STATE_GEOLOC:
			IWDG_reload();
			// Get position from GPS.
			lpuart_status = LPUART1_power_on();
			LPUART1_error_check();
			neom8n_status = NEOM8N_get_position(&spsws_ctx.position, SPSWS_GEOLOC_TIMEOUT_SECONDS, &spsws_ctx.geoloc_fix_duration_seconds);
			NEOM8N_error_check();
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
			// Turn radio TCXO off.
			sx1232_status = SX1232_tcxo(0);
			SX1232_error_check();
			// Disable HSE.
			SPSWS_clock_sleep();
			// Clear POR flag.
			spsws_ctx.flags.por = 0;
			// Store status byte in NVM.
			nvm_status = NVM_write_byte(NVM_ADDRESS_STATUS, spsws_ctx.status.raw_byte);
			NVM_error_check();
#ifdef CM
			// Re-start continuous measurements.
			WIND_reset_data();
			RAIN_reset_data();
			WIND_start_continuous_measure();
			RAIN_start_continuous_measure();
			NVIC_enable_interrupt(NVIC_IT_EXTI_4_15);
#endif
			// Clear RTC flags.
			RTC_clear_alarm_a_flag();
			RTC_clear_alarm_b_flag();
			RTC_enable_alarm_a_interrupt();
			RTC_enable_alarm_b_interrupt();
			// Enter sleep mode.
			spsws_ctx.state = SPSWS_STATE_SLEEP;
			break;
		case SPSWS_STATE_SLEEP:
			IWDG_reload();
			// Enter sleep mode.
			PWR_enter_stop_mode();
			// Check RTC flags.
			if (RTC_get_alarm_b_flag() != 0) {
#ifdef CM
				// Call WIND callback.
				wind_status = WIND_measurement_period_callback();
				WIND_error_check();
#endif
				// Clear RTC flags.
				RTC_clear_alarm_b_flag();
			}
			if (RTC_get_alarm_a_flag() != 0) {
#ifdef CM
				// Stop continuous measurements.
				WIND_stop_continuous_measure();
				RAIN_stop_continuous_measure();
				NVIC_disable_interrupt(NVIC_IT_EXTI_4_15);
#endif
				// Disable RTC alarm interrupts.
				RTC_disable_alarm_a_interrupt();
				RTC_disable_alarm_b_interrupt();
				// Clear RTC flags.
				RTC_clear_alarm_a_flag();
				// Wake-up.
				spsws_ctx.state = SPSWS_STATE_WAKE_UP;
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
	// Local variables.
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Init board.
	SPSWS_init_context();
	SPSWS_init_hw();
	// Turn RF TCXO on.
	sx1232_status = SX1232_tcxo(1);
	SX1232_error_check();
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
