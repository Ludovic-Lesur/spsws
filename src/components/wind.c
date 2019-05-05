/*
 * wind.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#include "wind.h"

#include "exti.h"
#include "mapping.h"
#include "max11136.h"
#include "mode.h"
#include "nvic.h"
#include "spi.h"
#include "tim.h"
#include "tim_reg.h"
#include "usart.h"

#if (defined CM_RTC || defined ATM)

/*** WIND local macros ***/

// Speed count conversion ratio.
#ifdef WIND_VANE_ULTIMETER
#define WIND_SPEED_1HZ_TO_MH	2400
#endif
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define WIND_SPEED_1HZ_TO_MH	2400
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
	unsigned int wind_direction_degrees_average; // Average wind direction (°).
} WIND_Context;

/*** WIND local global variables ***/

static volatile WIND_Context wind_ctx;

/*** WIND functions ***/

/* INIT WIND VANE.
 * @param:	None.
 * @return:	None.
 */
void WIND_Init(void) {

	/* GPIO mapping selection */
	GPIO_WIND_SPEED = GPIO_DIO0;
#ifdef WIND_VANE_ULTIMETER
	GPIO_WIND_DIRECTION = GPIO_DIO1;
#endif

	/* Init GPIOs and EXTI */
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	GPIO_Configure(&GPIO_WIND_SPEED, Input, OpenDrain, LowSpeed, NoPullUpNoPullDown);
	EXTI_ConfigureInterrupt(&GPIO_WIND_SPEED, EXTI_TRIGGER_FALLING_EDGE);
#endif
#ifdef WIND_VANE_ULTIMETER
	// Wind speed.
	GPIO_Configure(&GPIO_WIND_SPEED, Input, OpenDrain, LowSpeed, NoPullUpNoPullDown);
	EXTI_ConfigureInterrupt(&GPIO_WIND_SPEED, EXTI_TRIGGER_RISING_EDGE);
	// Wind direction.
	GPIO_Configure(&GPIO_WIND_DIRECTION, Input, OpenDrain, LowSpeed, NoPullUpNoPullDown);
	EXTI_ConfigureInterrupt(&GPIO_WIND_DIRECTION, EXTI_TRIGGER_RISING_EDGE);
	// Init phase shift timer and associated variables.
	TIM2_Init(TIM2_MODE_WIND, 0);
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
	NVIC_EnableInterrupt(IT_TIM21);
	NVIC_EnableInterrupt(IT_EXTI_4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StopContinuousMeasure(void) {

	/* Disable required interrupts */
	NVIC_DisableInterrupt(IT_EXTI_4_15);
	NVIC_DisableInterrupt(IT_TIM21);

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
 * @param average_wind_direction_degrees:	Pointer to int that will contain average wind direction value in °.
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
	wind_ctx.wind_direction_degrees = 0;
#ifdef WIND_VANE_ULTIMETER
	wind_ctx.wind_direction_pwm_period = 0;
	wind_ctx.wind_direction_pwm_duty_cycle = 0;
#endif
	wind_ctx.wind_direction_data_count = 0;
	wind_ctx.wind_direction_degrees_average = 0;
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

		/* Wind direction */
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
		// Get analog value from ADC.
#ifdef HW1_0
		SPI1_PowerOn();
#endif
#ifdef HW2_0
		SPI2_PowerOn();
#endif
		MAX11136_PerformMeasurements();
#ifdef HW1_0
			SPI1_PowerOff();
#endif
#ifdef HW2_0
			SPI2_PowerOff();
#endif
		unsigned int wind_direction_12bits = 0;
		MAX11136_GetChannel(MAX11136_CHANNEL_WIND_DIRECTION, &wind_direction_12bits);
		// Convert voltage to direction (TBD).
		wind_ctx.wind_direction_degrees = 0;
#endif
		// Update average value.
		wind_ctx.wind_direction_degrees_average = ((wind_ctx.wind_direction_degrees_average * wind_ctx.wind_direction_data_count) + wind_ctx.wind_direction_degrees) / (wind_ctx.wind_direction_data_count + 1);
		wind_ctx.wind_direction_data_count++;

		/* Print data */
#ifdef ATM
		USARTx_SendString("Average speed = ");
		USARTx_SendValue(wind_ctx.wind_speed_mh_average, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("m/h ---- Peak speed = ");
		USARTx_SendValue(wind_ctx.wind_speed_mh_peak, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("m/h ---- Average direction = ");
		USARTx_SendValue(wind_ctx.wind_direction_degrees_average, USART_FORMAT_DECIMAL, 0);
		USARTx_SendString("°\n");
#endif

		/* Reset seconds counter */
		wind_ctx.wind_seconds_count = 0;
	}
}

#endif
