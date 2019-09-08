/*
 * wind.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#include "wind.h"

#include "exti.h"
#include "lptim.h"
#include "mapping.h"
#include "max11136.h"
#include "mode.h"
#include "nvic.h"
#include "rcc.h"
#include "spi.h"
#include "tim.h"
#include "tim_reg.h"
#include "usart.h"

#if (defined CM || defined ATM)

/*** WIND local macros ***/

// Speed count conversion ratio.
#define WIND_ANGLE_ERROR_VALUE		0xFFFFFFFF
#define WIND_DIRECTION_ERROR_VALUE	0xFF
#ifdef WIND_VANE_ULTIMETER
#define WIND_SPEED_1HZ_TO_MH		5400
#endif
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define WIND_SPEED_1HZ_TO_MH		2400
#define WIND_NUMBER_OF_DIRECTIONS	16 // Number of positions.
#define WIND_DIRECTION_PULL_UP_OHM	10000 // Pull-resistor value in Ohms.
#endif

/*** WIND local structures ***/

typedef struct {
	// Measurement period.
	unsigned char wind_seconds_count;
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
	unsigned int wind_direction_data_count;
	unsigned int wind_direction_degrees_average; // Average wind direction.
} WIND_Context;

/*** WIND local global variables ***/

static volatile WIND_Context wind_ctx;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
// Warning: resistor table should be sorted in ascending order.
static const unsigned int wind_vane_resistor_table_ohm[WIND_NUMBER_OF_DIRECTIONS] = {688, 891, 1000, 1410, 2200, 3140, 3900, 6570, 8200, 14120, 16000, 21880, 33000, 42120, 64900, 120000};
static unsigned int wind_vane_threshold_table_mv[WIND_NUMBER_OF_DIRECTIONS];
// Warning: angles have to be provided in the same order as the resistor table.
static const unsigned int wind_vane_angle_table_degrees[WIND_NUMBER_OF_DIRECTIONS] = {112, 67, 90, 157, 135, 202, 180, 22, 45, 247, 225, 337, 0, 292, 315, 270};
#endif

/*** WIND functions ***/

/* INIT WIND VANE.
 * @param:	None.
 * @return:	None.
 */
void WIND_Init(void) {

	/* Init data */
	WIND_ResetData();

	/* GPIO mapping selection */
	GPIO_WIND_SPEED = GPIO_DIO0;
#ifdef WIND_VANE_ULTIMETER
#ifdef HW1_0
	GPIO_WIND_DIRECTION = GPIO_DIO2;
#endif
#ifdef HW2_0
	GPIO_WIND_DIRECTION = GPIO_DIO1;
#endif
#endif

	/* Init GPIOs and EXTI */
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	GPIO_Configure(&GPIO_WIND_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureInterrupt(&GPIO_WIND_SPEED, EXTI_TRIGGER_FALLING_EDGE);
#endif
#ifdef WIND_VANE_ULTIMETER
	// Wind speed.
	GPIO_Configure(&GPIO_WIND_SPEED, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureInterrupt(&GPIO_WIND_SPEED, EXTI_TRIGGER_RISING_EDGE);
	// Wind direction.
	GPIO_Configure(&GPIO_WIND_DIRECTION, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureInterrupt(&GPIO_WIND_DIRECTION, EXTI_TRIGGER_RISING_EDGE);
#endif
}

/* START CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StartContinuousMeasure(void) {

	/* Reset second counter */
	wind_ctx.wind_seconds_count = 0;

#ifdef WIND_VANE_ULTIMETER
	/* Init TIM2 for phase shift operation */
	TIM2_Init(TIM2_MODE_WIND, 0);
	TIM2_Enable();
#endif

	/* Enable required interrupts */
	EXTI_ClearAllFlags();
	NVIC_EnableInterrupt(IT_EXTI_4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StopContinuousMeasure(void) {

	/* Disable required interrupts */
	NVIC_DisableInterrupt(IT_EXTI_4_15);

#ifdef WIND_VANE_ULTIMETER
	/* Stop TIM2 */
	TIM2_Stop();
	TIM2_Disable();
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
 * @param average_wind_direction_degrees:	Pointer to int that will contain average wind direction value in ï¿½.
 * @return:									None.
 */
void WIND_GetDirection(unsigned int* average_wind_direction_degrees) {
	(*average_wind_direction_degrees) = wind_ctx.wind_direction_degrees_average;
}

/* RESET WIND MEASUREMENT DATA.
 * @param:	None.
 * @return:	None.
 */
void WIND_ResetData(void) {

	/* Measurement period */
	wind_ctx.wind_seconds_count = 0;

	/* Wind speed */
	wind_ctx.wind_speed_edge_count = 0;
	wind_ctx.wind_speed_data_count = 0;
	wind_ctx.wind_speed_mh = 0;
	wind_ctx.wind_speed_mh_average = 0;
	wind_ctx.wind_speed_mh_peak = 0;

	/* Wind direction */
	wind_ctx.wind_direction_degrees = WIND_ANGLE_ERROR_VALUE;
#ifdef WIND_VANE_ULTIMETER
	wind_ctx.wind_direction_pwm_period = 0;
	wind_ctx.wind_direction_pwm_duty_cycle = 0;
#endif
	wind_ctx.wind_direction_data_count = 0;
	wind_ctx.wind_direction_degrees_average = WIND_DIRECTION_ERROR_VALUE;
}

/*** Wind utility functions ***/

/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE SPEED SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void WIND_SpeedEdgeCallback(void) {

	/* Wind speed */
	wind_ctx.wind_speed_edge_count++;

	/* Wind direction */
#ifdef WIND_VANE_ULTIMETER
	// Capture PWM period.
	TIM2_Stop();
	wind_ctx.wind_direction_pwm_period = TIM2_GetCounter();
	// Compute direction
	if ((wind_ctx.wind_direction_pwm_period > 0) && (wind_ctx.wind_direction_pwm_duty_cycle <= wind_ctx.wind_direction_pwm_period)) {
		wind_ctx.wind_direction_degrees = (wind_ctx.wind_direction_pwm_duty_cycle * 360) / (wind_ctx.wind_direction_pwm_period);
	}
	// Start new cycle.
	TIM2_Start();
#endif
}

#ifdef WIND_VANE_ULTIMETER
/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE DIRECTION SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void WIND_DirectionEdgeCallback(void) {

	/* Capture PWM duty cycle */
	wind_ctx.wind_direction_pwm_duty_cycle = TIM2_GetCounter();
}
#endif

#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
/* CONVERT OUTPUT VOLTAGE TO WIND VANE ANGLE.
 * @param vcc_mv:			Voltage divider supply in mV.
 * @param direction_mv:		Voltage divider output voltage in mV.
 * @return wind_vane_angle:	Corresponding angle in degrees.
 */
unsigned int WIND_VoltageToAngle(unsigned int vcc_mv, unsigned int direction_mv) {

	/* Local variables */
	unsigned int wind_vane_angle = WIND_ANGLE_ERROR_VALUE;
	unsigned char idx = 0;
	unsigned int lower_voltage_mv = 0;
	unsigned int upper_voltage_mv = 0;

	/* Compute threshold table */
	for (idx=0 ; idx<(WIND_NUMBER_OF_DIRECTIONS-1) ; idx++) {
		// Lower voltage.
		lower_voltage_mv = (wind_vane_resistor_table_ohm[idx] * vcc_mv) / (wind_vane_resistor_table_ohm[idx] + WIND_DIRECTION_PULL_UP_OHM);
		upper_voltage_mv = (wind_vane_resistor_table_ohm[idx+1] * vcc_mv) / (wind_vane_resistor_table_ohm[idx+1] + WIND_DIRECTION_PULL_UP_OHM);
		// Compute average.
		wind_vane_threshold_table_mv[idx] = (lower_voltage_mv + upper_voltage_mv) / 2;
	}
	// Last threshold is Vcc.
	wind_vane_threshold_table_mv[WIND_NUMBER_OF_DIRECTIONS-1] = vcc_mv;

	/* Compute angle */
	for (idx=0 ; idx<WIND_NUMBER_OF_DIRECTIONS ; idx++) {
		if (direction_mv < wind_vane_threshold_table_mv[idx]) {
			wind_vane_angle = wind_vane_angle_table_degrees[idx];
			break;
		}
	}
	return wind_vane_angle;
}
#endif

/* FUNCTION CALLED BY TIM21 INTERRUPT HANDLER WHEN THE MEASUREMENT PERIOD IS REACHED.
 * @param:	None.
 * @return:	None.
 */
void WIND_MeasurementPeriodCallback(void) {

	/* Update counter */
	wind_ctx.wind_seconds_count++;

	/* Update wind measurements if period is reached */
	if (wind_ctx.wind_seconds_count == WIND_MEASUREMENT_PERIOD_SECONDS) {

		/* Wind speed */
		// Compute new value.
		wind_ctx.wind_speed_mh = (wind_ctx.wind_speed_edge_count * WIND_SPEED_1HZ_TO_MH) / (WIND_MEASUREMENT_PERIOD_SECONDS);
		wind_ctx.wind_speed_edge_count = 0;
		// Update peak value if required.
		if (wind_ctx.wind_speed_mh > wind_ctx.wind_speed_mh_peak) {
			wind_ctx.wind_speed_mh_peak = wind_ctx.wind_speed_mh;
		}
		// Update average value.
		wind_ctx.wind_speed_mh_average = ((wind_ctx.wind_speed_mh_average * wind_ctx.wind_speed_data_count) + wind_ctx.wind_speed_mh) / (wind_ctx.wind_speed_data_count + 1);
		wind_ctx.wind_speed_data_count++;

		/* Wind direction (only if there is wind) */
		if (wind_ctx.wind_speed_mh > 0) {
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
			// Internal 16MHz clock.
			RCC_SwitchToHsi();
			// Timers.
			TIM21_Start();
			TIM22_Start();
			LPTIM1_Enable();
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
			TIM21_Disable();
			TIM22_Disable();
			LPTIM1_Disable();
			// Get 12-bits result.
			unsigned int bandgap_12bits = 0;
			unsigned int wind_direction_12bits = 0;
			MAX11136_GetChannel(MAX11136_CHANNEL_BANDGAP, &bandgap_12bits);
			MAX11136_GetChannel(MAX11136_CHANNEL_WIND_DIRECTION, &wind_direction_12bits);
			// Convert to mV.
			unsigned int wind_vcc_mv = (MAX11136_BANDGAP_VOLTAGE_MV * MAX11136_FULL_SCALE) / bandgap_12bits;
			unsigned int wind_direction_mv = (wind_direction_12bits * MAX11136_BANDGAP_VOLTAGE_MV) / (bandgap_12bits);
			// Convert voltage to direction (TBD).
			wind_ctx.wind_direction_degrees = WIND_VoltageToAngle(wind_vcc_mv, wind_direction_mv);
#endif
			// Update average value if direction is valid.
			if (wind_ctx.wind_direction_degrees != WIND_ANGLE_ERROR_VALUE) {
				wind_ctx.wind_direction_degrees_average = ((wind_ctx.wind_direction_degrees_average * wind_ctx.wind_direction_data_count) + wind_ctx.wind_direction_degrees) / (wind_ctx.wind_direction_data_count + 1);
				wind_ctx.wind_direction_data_count++;
			}
		}
		else {
			wind_ctx.wind_direction_degrees = WIND_ANGLE_ERROR_VALUE;
		}

		/* Print data */
#ifdef ATM
		USARTx_SendString("Speed=");
		USARTx_SendValue(wind_ctx.wind_speed_mh, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("m/h Direction=");
		USARTx_SendValue(wind_ctx.wind_direction_degrees, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("°\n");
#endif

		/* Reset seconds counter */
		wind_ctx.wind_seconds_count = 0;
	}
}

#endif
