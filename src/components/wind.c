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
#include "rcc.h"
#include "spi.h"

#if (defined CM || defined ATM)

/*** WIND local macros ***/

// Speed count conversion ratio.
#ifdef WIND_VANE_ULTIMETER
#define WIND_SPEED_1HZ_TO_MH				5400
#endif
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define WIND_SPEED_1HZ_TO_MH				2400
#define WIND_NUMBER_OF_DIRECTIONS			16 // Number of positions.
#endif

/*** WIND local structures ***/

typedef struct {
	// Measurements period.
	unsigned char speed_seconds_count;
	unsigned char direction_seconds_count;
	// Wind speed.
	unsigned int speed_edge_count;
	unsigned int speed_data_count;
	unsigned int speed_mh; // Current value.
	unsigned int speed_mh_average; // Average wind speed (m/h).
	unsigned int speed_mh_peak; // Peak wind speed (m/h).
	// Wind direction.
	unsigned int direction_degrees; // Current value.
#ifdef WIND_VANE_ULTIMETER
	unsigned int direction_pwm_period; // TIM2 counter value between 2 edge interrupts on wind speed input.
	unsigned int direction_pwm_duty_cycle; // TIM2 counter value when edge interrupt detected on wind direction input.
#endif
	signed int direction_x; // x coordinate of wind direction trend point.
	signed int direction_y; // y coordinate of wind direction trend point.
} WIND_context_t;

/*** WIND local global variables ***/

static volatile WIND_context_t wind_ctx;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
// Rp = 10k (pull-up resistor).
// Rw = 688, 891, 1000, 1410, 2200, 3140, 3900, 6570, 8200, 14120, 16000, 21880, 33000, 42120, 64900 and 120000.
// Resistor divider ratio values = 1000 * ((Rw) / (Rw + Rp)).
// The following table gives the mean between two consecutive ratios (used as threshold). It must be sorted in ascending order.
static const unsigned int WIND_DIRECTION_RESISTOR_DIVIDER_RATIO_THRESHOLD_TABLE[WIND_NUMBER_OF_DIRECTIONS] = {73, 86, 107, 152, 210, 260, 338, 424, 518, 600, 651, 727, 788, 837, 895, 1000};
// Angle table (must be mapped on the ratio table: angle[i] = angle for ratio[i]).
static const unsigned int WIND_DIRECTION_ANGLE_TABLE[WIND_NUMBER_OF_DIRECTIONS] = {112, 67, 90, 157, 135, 202, 180, 22, 45, 247, 225, 337, 0, 292, 315, 270};
#endif

/*** WIND local functions ***/

#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
/* CONVERT OUTPUT VOLTAGE TO WIND VANE ANGLE.
 * @param vcc_mv:		Voltage divider supply in mV.
 * @param direction_mv:	Voltage divider output voltage in mV.
 * @return:				None.
 */
static void WIND_voltage_to_angle(unsigned int ratio) {
	// Local variables.
	unsigned char idx = 0;
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

/* INIT WIND VANE.
 * @param:	None.
 * @return:	None.
 */
void WIND_init(void) {
	// Init GPIOs and EXTI.
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	GPIO_configure(&GPIO_DIO0, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_DIO0, EXTI_TRIGGER_FALLING_EDGE);
#endif
#ifdef WIND_VANE_ULTIMETER
	// Wind speed.
	GPIO_configure(&GPIO_DIO0, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_DIO0, EXTI_TRIGGER_RISING_EDGE);
	// Wind direction.
	GPIO_configure(&GPIO_DIO1, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_DIO1, EXTI_TRIGGER_RISING_EDGE);
#endif
	// Reset data.
	WIND_reset_data();
	// Set interrupt priority.
	NVIC_set_priority(NVIC_IT_EXTI_4_15, 0);
}

/* START CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_start_continuous_measure(void) {
	// Reset second counters.
	wind_ctx.speed_seconds_count = 0;
	wind_ctx.direction_seconds_count = 0;
#ifdef WIND_VANE_ULTIMETER
	// Init phase shift timers: TBD.
	LPTIM1_enable();
#endif
	// Enable required interrupts.
	EXTI_clear_all_flags();
	NVIC_enable_interrupt(NVIC_IT_EXTI_4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_stop_continuous_measure(void) {
	// Disable required interrupts.
	NVIC_disable_interrupt(NVIC_IT_EXTI_4_15);
#ifdef WIND_VANE_ULTIMETER
	// Stop phase shift timer.
	LPTIM1_disable();
#endif
}

/* GET AVERAGE AND PEAK WIND SPEED VALUES SINCE LAST MEASUREMENT START.
 * @param average_speed_mh:	Pointer to int that will contain average wind speed value in m/h.
 * @param peak_speed_mh:		Pointer to int that will contain peak wind speed value in m/h.
 * @return:							None.
 */
void WIND_get_speed(unsigned int* average_speed_mh, unsigned int* peak_speed_mh) {
	(*average_speed_mh) = wind_ctx.speed_mh_average;
	(*peak_speed_mh) = wind_ctx.speed_mh_peak;
}

/* GET AVERAGE AVERAGE WIND DIRECTION VALUE SINCE LAST MEASUREMENT START.
 * @param average_direction_degrees:	Pointer to int that will contain average wind direction value in degrees.
 * @return status:						Function execution status.
 */
WIND_status_t WIND_get_direction(unsigned int* average_direction_degrees) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
	unsigned int local_result = 0;
	// Compute trend point angle (considered as average wind direction).
	local_result = MATH_atan2(wind_ctx.direction_x, wind_ctx.direction_y);
	if (local_result == MATH_ERROR_VALUE) {
		status = WIND_ERROR_MATH;
		goto errors;
	}
	(*average_direction_degrees) = local_result;
errors:
	return status;
}

/* RESET WIND MEASUREMENT DATA.
 * @param:	None.
 * @return:	None.
 */
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

/*** WIND utility functions ***/

/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE SPEED SIGNAL.
 * @param:	None.
 * @return:	None.
 */
WIND_status_t WIND_speed_edge_callback(void) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
	// Wind speed.
	wind_ctx.speed_edge_count++;
	// Wind direction.
#ifdef WIND_VANE_ULTIMETER
	// Capture PWM period.
	LPTIM1_stop();
	wind_ctx.direction_pwm_period = LPTIM1_get_counter();
	// Compute direction
	if ((wind_ctx.direction_pwm_period > 0) && (wind_ctx.direction_pwm_duty_cycle <= wind_ctx.direction_pwm_period)) {
		wind_ctx.direction_degrees = (wind_ctx.direction_pwm_duty_cycle * 360) / (wind_ctx.direction_pwm_period);
	}
	// Start new cycle (LPTIM1 status not checked).
	status = LPTIM1_start();
#endif
	return status;
}

#ifdef WIND_VANE_ULTIMETER
/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE DIRECTION SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void WIND_direction_edge_callback(void) {
	// Capture PWM duty cycle.
	wind_ctx.direction_pwm_duty_cycle = LPTIM1_get_counter();
}
#endif

/* FUNCTION CALLED BY TIM21 INTERRUPT HANDLER WHEN THE MEASUREMENT PERIOD IS REACHED.
 * @param:			None.
 * @return status:	Function execution status.
 */
WIND_status_t WIND_measurement_period_callback(void) {
	// Local variables.
	WIND_status_t status = WIND_SUCCESS;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#ifdef HW1_0
	SPI_status_t spi1_status = SPI_SUCCESS;
#endif
#ifdef HW2_0
	SPI_status_t spi2_status = SPI_SUCCESS;
#endif
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	unsigned int wind_direction_ratio = 0;
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
			// Get direction from ADC.
#ifdef HW1_0
			SPI1_enable();
			spi1_status = SPI1_power_on();
			SPI1_status_check(WIND_ERROR_SPI);
#endif
#ifdef HW2_0
			SPI2_enable();
			spi2_status = SPI2_power_on();
			SPI2_status_check(WIND_ERROR_BASE_SPI);
#endif
			max11136_status = MAX11136_perform_measurements();
			MAX11136_status_check(WIND_ERROR_BASE_MAX11136);

#ifdef HW1_0
			spi1_status = SPI1_power_off();
			SPI1_status_check(WIND_ERROR_SPI);
			SPI1_disable();
#endif
#ifdef HW2_0
			spi2_status = SPI2_power_off();
			SPI2_status_check(WIND_ERROR_BASE_SPI);
			SPI2_disable();
#endif
			// Get 12-bits result.
			max11136_status = MAX11136_get_data(MAX11136_DATA_WIND_DIRECTION_RATIO, &wind_direction_ratio);
			MAX11136_status_check(WIND_ERROR_BASE_MAX11136)
			// Convert voltage to direction.
			WIND_voltage_to_angle(wind_direction_ratio);
#endif
			// Add new vector: x=speed*cos(angle) and y=speed*sin(angle).
			wind_ctx.direction_x += (wind_ctx.speed_mh / 1000) * (signed int) MATH_COS_TABLE[wind_ctx.direction_degrees];
			wind_ctx.direction_y += (wind_ctx.speed_mh / 1000) * (signed int) MATH_SIN_TABLE[wind_ctx.direction_degrees];
#ifdef ATM
			AT_print_wind_direction(wind_ctx.direction_degrees, wind_ctx.direction_x, wind_ctx.direction_y);
#endif
		}
		// Reset seconds counter.
		wind_ctx.direction_seconds_count = 0;
	}
errors:
	return status;
}

#endif
