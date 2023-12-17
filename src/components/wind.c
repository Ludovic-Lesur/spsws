/*
 * wind.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#include "wind.h"

#include "at.h"
#include "exti.h"
#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "max11136.h"
#include "mode.h"
#include "nvic.h"
#include "power.h"
#include "rcc.h"
#include "types.h"

/*** WIND local macros ***/

// DIOs mapping.
#define WIND_GPIO_SPEED				GPIO_DIO0
#ifdef WIND_VANE_ULTIMETER
#define WIND_GPIO_DIRECTION			GPIO_DIO1
#endif
// Speed count conversion ratio.
#ifdef WIND_VANE_ULTIMETER
#define WIND_SPEED_1HZ_TO_MH		5400
#endif
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define WIND_SPEED_1HZ_TO_MH		2400
#define WIND_NUMBER_OF_DIRECTIONS	16 // Number of positions.
#endif

/*** WIND local structures ***/

/*******************************************************************/
typedef struct {
	// Measurements period.
	uint8_t speed_seconds_count;
	uint8_t direction_seconds_count;
	// Wind speed.
	uint32_t speed_edge_count;
	uint32_t speed_data_count;
	uint32_t speed_mh; // Current value.
	uint32_t speed_mh_average; // Average wind speed (m/h).
	uint32_t speed_mh_peak; // Peak wind speed (m/h).
	// Wind direction.
	uint32_t direction_degrees; // Current value.
#ifdef WIND_VANE_ULTIMETER
	uint32_t direction_pwm_period; // TIM2 counter value between 2 edge interrupts on wind speed input.
	uint32_t direction_pwm_duty_cycle; // TIM2 counter value when edge interrupt detected on wind direction input.
#endif
	int32_t direction_x; // x coordinate of wind direction trend point.
	int32_t direction_y; // y coordinate of wind direction trend point.
} WIND_context_t;

/*** WIND local global variables ***/

#ifdef SPSWS_WIND_MEASUREMENT
static volatile WIND_context_t wind_ctx;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
// Rp = 10k (pull-up resistor).
// Rw = 688, 891, 1000, 1410, 2200, 3140, 3900, 6570, 8200, 14120, 16000, 21880, 33000, 42120, 64900 and 120000.
// Resistor divider ratio values = 1000 * ((Rw) / (Rw + Rp)).
// The following table gives the mean between two consecutive ratios (used as threshold). It must be sorted in ascending order.
static const uint32_t WIND_DIRECTION_RESISTOR_DIVIDER_RATIO_THRESHOLD_TABLE[WIND_NUMBER_OF_DIRECTIONS] = {73, 86, 107, 152, 210, 260, 338, 424, 518, 600, 651, 727, 788, 837, 895, 1000};
// Angle table (must be mapped on the ratio table: angle[i] = angle for ratio[i]).
static const uint32_t WIND_DIRECTION_ANGLE_TABLE[WIND_NUMBER_OF_DIRECTIONS] = {112, 67, 90, 157, 135, 202, 180, 22, 45, 247, 225, 337, 0, 292, 315, 270};
#endif
#endif

/*** WIND local functions ***/

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
static void _WIND_speed_edge_callback(void) {
	// Wind speed.
	wind_ctx.speed_edge_count++;
	// Wind direction.
#ifdef WIND_VANE_ULTIMETER
	// Capture PWM period.
	// TODOS
	// Compute direction
	if ((wind_ctx.direction_pwm_period > 0) && (wind_ctx.direction_pwm_duty_cycle <= wind_ctx.direction_pwm_period)) {
		wind_ctx.direction_degrees = (wind_ctx.direction_pwm_duty_cycle * 360) / (wind_ctx.direction_pwm_period);
	}
	// Start new cycle.
	// TODO
#endif
}
#endif

#if (defined SPSWS_WIND_MEASUREMENT) && (defined WIND_VANE_ULTIMETER)
/*******************************************************************/
static void _WIND_direction_edge_callback(void) {
	// Capture PWM duty cycle.
	// TODO
}
#endif

#if (defined SPSWS_WIND_MEASUREMENT) && (defined WIND_VANE_ARGENT_DATA_SYSTEMS)
/*******************************************************************/
static void _WIND_voltage_to_angle(uint32_t ratio) {
	// Local variables.
	uint8_t idx = 0;
	// Get corresponding angle.
	for (idx=0 ; idx<WIND_NUMBER_OF_DIRECTIONS ; idx++) {
		if (ratio < WIND_DIRECTION_RESISTOR_DIVIDER_RATIO_THRESHOLD_TABLE[idx]) {
			// Update current angle and table index.
			wind_ctx.direction_degrees = WIND_DIRECTION_ANGLE_TABLE[idx];
			break;
		}
	}
}
#endif

/*** WIND functions ***/

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
void WIND_init(void) {
	// Init GPIOs and EXTI.
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	GPIO_configure(&WIND_GPIO_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&WIND_GPIO_SPEED, EXTI_TRIGGER_FALLING_EDGE, &_WIND_speed_edge_callback);
#endif
#ifdef WIND_VANE_ULTIMETER
	// Wind speed.
	GPIO_configure(&WIND_GPIO_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&WIND_GPIO_SPEED, EXTI_TRIGGER_RISING_EDGE, &_WIND_speed_edge_callback);
	// Wind direction.
	GPIO_configure(&WIND_GPIO_DIRECTION, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&WIND_GPIO_DIRECTION, EXTI_TRIGGER_RISING_EDGE, &_WIND_direction_edge_callback);
#endif
	// Reset data.
	WIND_reset_data();
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
void WIND_start_continuous_measure(void) {
	// Reset second counters.
	wind_ctx.speed_seconds_count = 0;
	wind_ctx.direction_seconds_count = 0;
#ifdef WIND_VANE_ULTIMETER
	// Init phase shift timers:
	// TODO
#endif
	// Enable interrupts.
	EXTI_clear_flag((EXTI_line_t) WIND_GPIO_SPEED.pin);
#ifdef WIND_VANE_ULTIMETER
	EXTI_clear_flag((EXTI_line_t) WIND_GPIO_DIRECTION.pin);
#endif
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_4_15, NVIC_PRIORITY_EXTI_4_15);
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
void WIND_stop_continuous_measure(void) {
	// Disable interrupts.
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
#ifdef WIND_VANE_ULTIMETER
	// Release phase shift timer.
	// TODO
#endif
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
void WIND_reset_data(void) {
	// Measurement periods.
	wind_ctx.speed_seconds_count = 0;
	wind_ctx.direction_seconds_count = 0;
	// Wind speed.
	wind_ctx.speed_edge_count = 0;
	wind_ctx.speed_data_count = 0;
	wind_ctx.speed_mh = 0;
	wind_ctx.speed_mh_average = 0;
	wind_ctx.speed_mh_peak = 0;
	// Wind direction.
	wind_ctx.direction_degrees = 0;
#ifdef WIND_VANE_ULTIMETER
	wind_ctx.direction_pwm_period = 0;
	wind_ctx.direction_pwm_duty_cycle = 0;
#endif
	wind_ctx.direction_x = 0;
	wind_ctx.direction_y = 0;
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
WIND_status_t WIND_tick_second(void) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	POWER_status_t power_status = POWER_SUCCESS;
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	uint32_t wind_direction_ratio = 0;
#endif
	// Update counters.
	wind_ctx.speed_seconds_count++;
	wind_ctx.direction_seconds_count++;
	// Update wind speed if period is reached.
	if (wind_ctx.speed_seconds_count >= WIND_SPEED_MEASUREMENT_PERIOD_SECONDS) {
		// Compute new value.
		wind_ctx.speed_mh = (wind_ctx.speed_edge_count * WIND_SPEED_1HZ_TO_MH) / (WIND_SPEED_MEASUREMENT_PERIOD_SECONDS);
		wind_ctx.speed_edge_count = 0;
		// Update peak value if required.
		if (wind_ctx.speed_mh > wind_ctx.speed_mh_peak) {
			wind_ctx.speed_mh_peak = wind_ctx.speed_mh;
		}
		// Update average value.
		wind_ctx.speed_mh_average = ((wind_ctx.speed_mh_average * wind_ctx.speed_data_count) + wind_ctx.speed_mh) / (wind_ctx.speed_data_count + 1);
		wind_ctx.speed_data_count++;
		// Reset seconds counter.
		wind_ctx.speed_seconds_count = 0;
#ifdef ATM
		AT_print_wind_speed(wind_ctx.speed_mh);
#endif
	}
	// Update wind direction if period is reached.
	if (wind_ctx.direction_seconds_count >= WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS) {
		// Compute direction only if there is wind.
		if ((wind_ctx.speed_mh / 1000) > 0) {
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
			// Turn external ADC on.
			power_status = POWER_enable(POWER_DOMAIN_ANALOG_EXTERNAL, LPTIM_DELAY_MODE_STOP);
			POWER_exit_error(WIND_ERROR_BASE_POWER);
			// Get direction from ADC.
			max11136_status = MAX11136_perform_measurements();
			MAX11136_exit_error(WIND_ERROR_BASE_MAX11136);
			// Turn external ADC off.
			power_status = POWER_disable(POWER_DOMAIN_ANALOG_EXTERNAL);
			POWER_exit_error(WIND_ERROR_BASE_POWER);
			// Get 12-bits result.
			max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_WIND_DIRECTION_RATIO, &wind_direction_ratio);
			MAX11136_exit_error(WIND_ERROR_BASE_MAX11136);
			// Convert voltage to direction.
			_WIND_voltage_to_angle(wind_direction_ratio);
#endif
			// Add new vector: x=speed*cos(angle) and y=speed*sin(angle).
			wind_ctx.direction_x += (wind_ctx.speed_mh / 1000) * (int32_t) MATH_COS_TABLE[wind_ctx.direction_degrees];
			wind_ctx.direction_y += (wind_ctx.speed_mh / 1000) * (int32_t) MATH_SIN_TABLE[wind_ctx.direction_degrees];
#ifdef ATM
			AT_print_wind_direction(wind_ctx.direction_degrees, wind_ctx.direction_x, wind_ctx.direction_y);
#endif
		}
		// Reset seconds counter.
		wind_ctx.direction_seconds_count = 0;
	}
	return status;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
errors:
	POWER_disable(POWER_DOMAIN_ANALOG_EXTERNAL);
#endif
	return status;
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
WIND_status_t WIND_get_speed(uint32_t* average_speed_mh, uint32_t* peak_speed_mh) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
	// Check parameters.
	if ((average_speed_mh == NULL) || (peak_speed_mh == NULL)) {
		status = WIND_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*average_speed_mh) = wind_ctx.speed_mh_average;
	(*peak_speed_mh) = wind_ctx.speed_mh_peak;
errors:
	return status;
}
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*******************************************************************/
WIND_status_t WIND_get_direction(uint32_t* average_direction_degrees) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	// Check parameters.
	if (average_direction_degrees == NULL) {
		status = WIND_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Compute trend point angle (considered as average wind direction).
	math_status = MATH_atan2(wind_ctx.direction_x, wind_ctx.direction_y, average_direction_degrees);
	MATH_exit_error(WIND_ERROR_BASE_MATH);
errors:
	return status;
}
#endif
