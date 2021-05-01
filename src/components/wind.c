/*
 * wind.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#include "wind.h"

#include "exti.h"
#include "mapping.h"
#include "math.h"
#include "max11136.h"
#include "mode.h"
#include "nvic.h"
#include "rcc.h"
#include "spi.h"
#include "usart.h"

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
	unsigned char wind_speed_seconds_count;
	unsigned char wind_direction_seconds_count;
	// Wind speed.
	unsigned int wind_speed_edge_count;
	unsigned int wind_speed_data_count;
	unsigned int wind_speed_mh; // Current value.
	unsigned int wind_speed_mh_average; // Average wind speed (m/h).
	unsigned int wind_speed_mh_peak; // Peak wind speed (m/h).
	// Wind direction.
	unsigned int wind_direction_degrees; // Current value.
#ifdef WIND_VANE_ULTIMETER
	unsigned int wind_direction_pwm_period; // TIM2 counter value between 2 edge interrupts on wind speed input.
	unsigned int wind_direction_pwm_duty_cycle; // TIM2 counter value when edge interrupt detected on wind direction input.
#endif
	signed int wind_direction_x; // x coordinate of wind direction trend point.
	signed int wind_direction_y; // y coordinate of wind direction trend point.
} WIND_Context;

/*** WIND local global variables ***/

static volatile WIND_Context wind_ctx;
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

/* COMPUTE ABSOLUTE VALUE.
 * @param x:	Parameter.
 * @return:		|x|.
 */
static unsigned int WIND_Abs(signed int x) {
	unsigned int result = 0;
	if (x > 0) {
		result = x;
	}
	if (x < 0) {
		result = (-1) * x;
	}
	return result;
}

/* COMPUTE THE TREND POINT DIRECTION (ATAN2 FUNCTION).
 * @param x:	x coordinate of the trend point.
 * @param y:	y coordinate of the trend point.
 * @return:		Angle of the point (x,y).
 */
static unsigned int WIND_Atan2(signed int x, signed int y) {
	// Check x and y are not null.
	unsigned int alpha = WIND_DIRECTION_ERROR_VALUE;
	if ((x != 0) || (y != 0)) {
		// Scale x and y to avoid overflow.
		signed int local_x = x;
		signed int local_y = y;
		while ((WIND_Abs(local_x) > 10000) || (WIND_Abs(local_y) > 10000)) {
			local_x = local_x >> 1;
			local_y = local_y >> 1;
		}
		// Compute atan2 function.
		unsigned int abs_x = WIND_Abs(local_x);
		unsigned int abs_y = WIND_Abs(local_y);
		// Use the quotient within [-1,1]
		if (abs_x >= abs_y) {
			// Use arctan approximation: arctan(z)=(pi/4)*z.
			alpha = (((45 * abs_y) << 10) / (abs_x)) >> 10; // Quadrant 1.
			// Add offset depending on quadrant.
			if ((x > 0) && (y < 0)) {
				// Quadrant 8.
				alpha = (360 - alpha);
			}
			if (x < 0) {
				if (y > 0) {
					// Quadrant 4.
					alpha = (180 - alpha);
				}
				else {
					// Quadrant 5.
					alpha = (180 + alpha);
				}
			}
		}
		else {
			// Use arctan approximation: arctan(z)=(pi/4)*z.
			alpha = (((45 * abs_x) << 10) / (abs_y)) >> 10;
			// Add offset depending on quadrant and arctan(1/z)=+/-90-arctan(z).
			if (x > 0) {
				if (y > 0) {
					// Quadrant 2.
					alpha = (90 - alpha);
				}
				else {
					// Quadrant 7.
					alpha = (270 + alpha);
				}
			}
			else {
				if (y > 0) {
					// Quadrant 3.
					alpha = (90 + alpha);
				}
				else {
					// Quadrant 6.
					alpha = (270 - alpha);
				}
			}
		}
		// Ensure angle is in [0,359] range.
		alpha = (alpha % 360);
	}
	return (alpha);
}

#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
/* CONVERT OUTPUT VOLTAGE TO WIND VANE ANGLE.
 * @param vcc_mv:		Voltage divider supply in mV.
 * @param direction_mv:	Voltage divider output voltage in mV.
 * @return:				None.
 */
static void WIND_VoltageToAngle(unsigned int direction_12bits) {
	// Reset result.
	wind_ctx.wind_direction_degrees = WIND_DIRECTION_ERROR_VALUE;

	/* Compute ratio */
	unsigned int ratio = (direction_12bits * 1000) / (MAX11136_FULL_SCALE);

	/* Get corresponding angle */
	unsigned char idx = 0;
	for (idx=0 ; idx<WIND_NUMBER_OF_DIRECTIONS ; idx++) {
		if (ratio < WIND_DIRECTION_RESISTOR_DIVIDER_RATIO_THRESHOLD_TABLE[idx]) {
			// Update current angle and table index.
			wind_ctx.wind_direction_degrees = WIND_DIRECTION_ANGLE_TABLE[idx];
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
void WIND_Init(void) {
	// GPIO mapping selection.
	GPIO_WIND_SPEED = GPIO_DIO0;
#ifdef WIND_VANE_ULTIMETER
#ifdef HW1_0
	GPIO_WIND_DIRECTION = GPIO_DIO2;
#endif
#ifdef HW2_0
	GPIO_WIND_DIRECTION = GPIO_DIO1;
#endif
#endif
	// Init GPIOs and EXTI.
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	GPIO_Configure(&GPIO_WIND_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureGpio(&GPIO_WIND_SPEED, EXTI_TRIGGER_FALLING_EDGE);
#endif
#ifdef WIND_VANE_ULTIMETER
	// Wind speed.
	GPIO_Configure(&GPIO_WIND_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureGpio(&GPIO_WIND_SPEED, EXTI_TRIGGER_RISING_EDGE);
	// Wind direction.
	GPIO_Configure(&GPIO_WIND_DIRECTION, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureGpio(&GPIO_WIND_DIRECTION, EXTI_TRIGGER_RISING_EDGE);
#endif
}

/* START CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StartContinuousMeasure(void) {
	// Reset second counters.
	wind_ctx.wind_speed_seconds_count = 0;
	wind_ctx.wind_direction_seconds_count = 0;
#ifdef WIND_VANE_ULTIMETER
	// Init phase shift timers: TBD.
#endif
	// Enable required interrupts.
	EXTI_ClearAllFlags();
	NVIC_SetPriority(NVIC_IT_EXTI_4_15, 0);
	NVIC_EnableInterrupt(NVIC_IT_EXTI_4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StopContinuousMeasure(void) {
	// Disable required interrupts.
	NVIC_DisableInterrupt(NVIC_IT_EXTI_4_15);
#ifdef WIND_VANE_ULTIMETER
	// Stop phase shift timers: TBD.
#endif
#ifdef ATM
	LPTIM1_Init(LPTIM_MODE_DELAY);
#endif
}

/* GET AVERAGE AND PEAK WIND SPEED VALUES SINCE LAST MEASUREMENT START.
 * @param average_wind_speed_mh:	Pointer to int that will contain average wind speed value in m/h.
 * @param peak_wind_speed_mh:		Pointer to int that will contain peak wind speed value in m/h.
 * @return:							None.
 */
void WIND_GetSpeed(unsigned int* average_wind_speed_mh, unsigned int* peak_wind_speed_mh) {
	(*average_wind_speed_mh) = wind_ctx.wind_speed_mh_average;
	(*peak_wind_speed_mh) = wind_ctx.wind_speed_mh_peak;
}

/* GET AVERAGE AVERAGE WIND DIRECTION VALUE SINCE LAST MEASUREMENT START.
 * @param average_wind_direction_degrees:	Pointer to int that will contain average wind direction value in degrees.
 * @return:									None.
 */
void WIND_GetDirection(unsigned int* average_wind_direction_degrees) {
	// Compute trend point angle (considered as average wind direction).
	(*average_wind_direction_degrees) = WIND_Atan2(wind_ctx.wind_direction_x, wind_ctx.wind_direction_y);
}

/* RESET WIND MEASUREMENT DATA.
 * @param:	None.
 * @return:	None.
 */
void WIND_ResetData(void) {
	// Measurement periods.
	wind_ctx.wind_speed_seconds_count = 0;
	wind_ctx.wind_direction_seconds_count = 0;
	// Wind speed.
	wind_ctx.wind_speed_edge_count = 0;
	wind_ctx.wind_speed_data_count = 0;
	wind_ctx.wind_speed_mh = 0;
	wind_ctx.wind_speed_mh_average = 0;
	wind_ctx.wind_speed_mh_peak = 0;
	// Wind direction.
	wind_ctx.wind_direction_degrees = WIND_DIRECTION_ERROR_VALUE;
#ifdef WIND_VANE_ULTIMETER
	wind_ctx.wind_direction_pwm_period = 0;
	wind_ctx.wind_direction_pwm_duty_cycle = 0;
#endif
	wind_ctx.wind_direction_x = 0;
	wind_ctx.wind_direction_y = 0;
}

/*** WIND utility functions ***/

/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE SPEED SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void WIND_SpeedEdgeCallback(void) {
	// Wind speed.
	wind_ctx.wind_speed_edge_count++;
	// Wind direction.
#ifdef WIND_VANE_ULTIMETER
	// Capture PWM period.
	// --- Stop phase shift timer.
	// --- wind_ctx.wind_direction_pwm_period = Timer_GetCounter();
	// Compute direction
	if ((wind_ctx.wind_direction_pwm_period > 0) && (wind_ctx.wind_direction_pwm_duty_cycle <= wind_ctx.wind_direction_pwm_period)) {
		wind_ctx.wind_direction_degrees = (wind_ctx.wind_direction_pwm_duty_cycle * 360) / (wind_ctx.wind_direction_pwm_period);
	}
	// Start new cycle.
	// --- Restart phase shift timer.
#endif
}

#ifdef WIND_VANE_ULTIMETER
/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE DIRECTION SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void WIND_DirectionEdgeCallback(void) {
	// Capture PWM duty cycle.
	// wind_ctx.wind_direction_pwm_duty_cycle = Timer_GetCounter();
}
#endif

/* FUNCTION CALLED BY TIM21 INTERRUPT HANDLER WHEN THE MEASUREMENT PERIOD IS REACHED.
 * @param:	None.
 * @return:	None.
 */
void WIND_MeasurementPeriodCallback(void) {
	// Update counters.
	wind_ctx.wind_speed_seconds_count++;
	wind_ctx.wind_direction_seconds_count++;
	// Update wind speed if period is reached.
	if (wind_ctx.wind_speed_seconds_count >= WIND_SPEED_MEASUREMENT_PERIOD_SECONDS) {
		// Compute new value.
		wind_ctx.wind_speed_mh = (wind_ctx.wind_speed_edge_count * WIND_SPEED_1HZ_TO_MH) / (WIND_SPEED_MEASUREMENT_PERIOD_SECONDS);
		wind_ctx.wind_speed_edge_count = 0;
		// Update peak value if required.
		if (wind_ctx.wind_speed_mh > wind_ctx.wind_speed_mh_peak) {
			wind_ctx.wind_speed_mh_peak = wind_ctx.wind_speed_mh;
		}
		// Update average value.
		wind_ctx.wind_speed_mh_average = ((wind_ctx.wind_speed_mh_average * wind_ctx.wind_speed_data_count) + wind_ctx.wind_speed_mh) / (wind_ctx.wind_speed_data_count + 1);
		wind_ctx.wind_speed_data_count++;
		// Reset seconds counter.
		wind_ctx.wind_speed_seconds_count = 0;
#ifdef ATM
		// Print data.
		USARTx_SendString("speed=");
		USARTx_SendValue(wind_ctx.wind_speed_mh, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("m/h\n");
#endif
	}
	// Update wind direction if period is reached.
	if (wind_ctx.wind_direction_seconds_count >= WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS) {
		// Compute direction only if there is wind.
		if ((wind_ctx.wind_speed_mh / 1000) > 0) {
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
			// Internal 16MHz clock.
			RCC_SwitchToHsi();
			// --- Init Timers.
			// SPI.
#ifdef HW1_0
			SPI1_Enable();
			SPI1_PowerOn();
#endif
#ifdef HW2_0
			SPI2_Enable();
			SPI2_PowerOn();
#endif
			MAX11136_PerformMeasurements();
#ifdef HW1_0
			SPI1_PowerOff();
			SPI1_Disable();
#endif
#ifdef HW2_0
			SPI2_PowerOff();
			SPI2_Disable();
#endif
			// Turn timers off.
			// --- Stop Timers.
			// Get 12-bits result.
			unsigned int wind_direction_12bits = 0;
			MAX11136_GetChannel(MAX11136_CHANNEL_WIND_DIRECTION, &wind_direction_12bits);
			// Convert voltage to direction.
			WIND_VoltageToAngle(wind_direction_12bits);
#endif
			// Update trend point if direction is valid.
			if (wind_ctx.wind_direction_degrees != WIND_DIRECTION_ERROR_VALUE) {
				// Add new vector: x=speed*cos(angle) and y=speed*sin(angle).
				wind_ctx.wind_direction_x += (wind_ctx.wind_speed_mh / 1000) * MATH_COS_TABLE[wind_ctx.wind_direction_degrees];
				wind_ctx.wind_direction_y += (wind_ctx.wind_speed_mh / 1000) * MATH_SIN_TABLE[wind_ctx.wind_direction_degrees];
#ifdef ATM
				// Print data.
				USARTx_SendString("direction=");
				USARTx_SendValue(wind_ctx.wind_direction_degrees, USART_FORMAT_DECIMAL, 0);
				USARTx_SendString("d x=");
				if (wind_ctx.wind_direction_x < 0) {
					USARTx_SendString("-");
				}
				USARTx_SendValue(WIND_Abs(wind_ctx.wind_direction_x), USART_FORMAT_DECIMAL, 0);
				USARTx_SendString(" y=");
				if (wind_ctx.wind_direction_y < 0) {
					USARTx_SendString("-");
				}
				USARTx_SendValue(WIND_Abs(wind_ctx.wind_direction_y), USART_FORMAT_DECIMAL, 0);
				USARTx_SendString("\n");
#endif
			}
		}
		else {
			wind_ctx.wind_direction_degrees = WIND_DIRECTION_ERROR_VALUE;
		}
		// Reset seconds counter.
		wind_ctx.wind_direction_seconds_count = 0;
	}
}

#endif
