/*
 * wind.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#include "wind.h"

#include "exti_reg.h"
#include "max11136.h"
#include "nvic.h"
#include "spi.h"
#include "tim.h"
#include "tim_reg.h"
#include "usart.h"

/*** WIND local macros ***/

#define WIND_BUFFER_SIZE		(3600 / WIND_MEASUREMENT_PERIOD_SECONDS) // 1 hour buffer.
#define WIND_DIRECTION_ANALOG	// Use analog input for wind direction if defined, digital phase shift instead.

/*** WIND local structures ***/

typedef struct {
	// Measurement period.
	unsigned char wind_seconds_count;
	// Wind speed.
#ifndef WIND_DIRECTION_ANALOG
	unsigned int wind_speed_raw_counter; // TIM2 counter value between 2 edge interrupts on wind speed input.
#endif
	unsigned char wind_speed_count; // Current revolution counter value.
	unsigned char wind_speed_count_buf[WIND_BUFFER_SIZE]; // Stores revolution counter for each measurement period.
	unsigned int wind_speed_count_buf_idx;
	unsigned char wind_speed_kmh_average; // Average wind speed (km/h).
	unsigned char wind_speed_kmh_peak; // Peak wind speed (km/h).
	// Wind direction.
#ifndef WIND_DIRECTION_ANALOG
	unsigned int wind_direction_raw_counter; // TIM2 counter value when edge interrupt detected on wind direction input.
#endif
	unsigned char wind_direction_pourcent; // Current wind direction value (%).
	unsigned char wind_direction_pourcent_buf[WIND_BUFFER_SIZE]; // Stores wind direction (%) for each measurement period.
	unsigned int wind_direction_pourcent_buf_idx;
	unsigned char wind_direction_pourcent_average; // Average wind direction (%).
} WIND_Context;

/*** WIND local global variables ***/

volatile WIND_Context wind_ctx;

/*** WIND local functions ***/

#ifdef WIND_DIRECTION_ANALOG
/* CONVERT ADC 12-BITS RAW VOLTAGE INTO A WIND DIRECTION IN %.
 * @param:	None.
 * @return:	None.
 */
void WIND_ConvertDirectionVoltageToPourcent(void) {
	// Get ADC raw result.
	unsigned int wind_direction_mv = 0;
	MAX11136_GetChannelVoltage(MAX11136_CHANNEL_AIN0, &wind_direction_mv);
	// TBD transfer function:
	// wind_direction_raw_mv ----> wind_ctx.wind_direction_pourcent.
}
#endif

/* STORE CURRENT MEASUREMENTS IN BUFFERS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StoreMeasurements(void) {

	/* Wind speed */
	// Store current counter.
	wind_ctx.wind_speed_count_buf[wind_ctx.wind_speed_count_buf_idx] = wind_ctx.wind_speed_count;
	// Reset counter.
	wind_ctx.wind_speed_count = 0;
	// Increment index and manage roll-over.
	wind_ctx.wind_speed_count_buf_idx++;
	if (wind_ctx.wind_speed_count_buf_idx == WIND_BUFFER_SIZE) {
		wind_ctx.wind_speed_count_buf_idx = 0;
	}

	/* Wind direction */
#ifdef WIND_DIRECTION_ANALOG
	// Get wind direction from ADC.
	SPI1_PowerOn();
	MAX11136_PerformMeasurements();
	SPI1_PowerOff();
	// Convert voltage to %.
	WIND_ConvertDirectionVoltageToPourcent();
#endif
	// Store current direction.
	wind_ctx.wind_direction_pourcent_buf[wind_ctx.wind_direction_pourcent_buf_idx] = wind_ctx.wind_direction_pourcent;
	// Increment index and manage roll-over.
	wind_ctx.wind_direction_pourcent_buf_idx++;
	if (wind_ctx.wind_direction_pourcent_buf_idx == WIND_BUFFER_SIZE) {
		wind_ctx.wind_direction_pourcent_buf_idx = 0;
	}
}

/* TIM21 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_IRQHandler(void) {

	/* Clear flag */
	TIM21 -> SR &= ~(0b1 << 0); // UIF='0'.

	/* Update counter */
	wind_ctx.wind_seconds_count++;

	/* Store wind measurements if period reached */
	if (wind_ctx.wind_seconds_count == WIND_MEASUREMENT_PERIOD_SECONDS) {
		// Store data.
		WIND_StoreMeasurements();
		// Print data.
		unsigned char mean_speed = 0;
		unsigned char peak_speed = 0;
		unsigned char mean_direction = 0;
		WIND_GetAveragePeakWindSpeed(&mean_speed, &peak_speed);
		WIND_GetAverageWindDirection(&mean_direction);
		USART2_SendString("mean_speed=");
		USART2_SendValue(mean_speed, USART_FORMAT_DECIMAL, 0);
		USART2_SendString("km/h peak_speed=");
		USART2_SendValue(peak_speed, USART_FORMAT_DECIMAL, 0);
		USART2_SendString("km/h mean_direction=");
		USART2_SendValue(mean_direction, USART_FORMAT_DECIMAL, 0);
		USART2_SendString("\n");
		// Reset counter.
		wind_ctx.wind_seconds_count = 0;
	}
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI4_15_IRQHandler(void) {

	/* PA15 (DIO2) edge interrupt */
	if (((EXTI -> PR) & (0b1 << 15)) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << 15); // PIF15='1' (writing '1' clears the bit).
		// Rising edge on speed signal.
		wind_ctx.wind_speed_count++;
#ifndef WIND_DIRECTION_ANALOG
		// Capture period.
		TIM2_Stop();
		wind_ctx.wind_speed_raw_counter = TIM2_GetCounter();
		// Compute direction
		if ((wind_ctx.wind_speed_raw_counter > 0) && (wind_ctx.wind_direction_raw_counter <= wind_ctx.wind_speed_raw_counter)) {
			wind_ctx.wind_direction_pourcent = (wind_ctx.wind_direction_raw_counter * 100) / (wind_ctx.wind_speed_raw_counter);
		}
		// Start new cycle.
		TIM2_Start();
#endif
	}

#ifndef WIND_DIRECTION_ANALOG
	/* PA8 (DIO3) edge interrupt */
	if (((EXTI -> PR) & (0b1 << 8)) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << 8); // PIF8='1' (writing '1' clears the bit).
		// Rising edge on direction signal.
		wind_ctx.wind_direction_raw_counter = TIM2_GetCounter();
	}
#endif
}

/* COMPUTE AVERAGE AND PEAK WIND SPEED IN CURRENT BUFFER.
 * @param:	None.
 * @return:	None.
 */
void WIND_ComputeAveragePeakWindSpeed(void) {
	/* Compute average speed counts in buffer */
	unsigned int idx = 0;
	unsigned int max = 0;
	unsigned int sum = 0;
	unsigned int mean = 0;
	unsigned int number_of_elements = 0;
	for (idx=0 ; idx<WIND_BUFFER_SIZE ; idx++) {
		// Take valid values only.
		if (wind_ctx.wind_speed_count_buf[idx] != 0xFF) {
			sum += wind_ctx.wind_speed_count_buf[idx];
			number_of_elements++;
			if (wind_ctx.wind_speed_count_buf[idx] > max) {
				max = wind_ctx.wind_speed_count_buf[idx];
			}
		}
	}
	mean = (sum / number_of_elements);

	/* Convert value to km/h */
	wind_ctx.wind_speed_kmh_average = mean;
	wind_ctx.wind_speed_kmh_peak = max;
}

/* COMPUTE AVERAGE WIND DIRECTION IN CURRENT BUFFER.
 * @param:	None.
 * @return:	None.
 */
void WIND_ComputeAverageWindDirection(void) {
	/* Compute average speed counts in buffer */
	unsigned int idx = 0;
	unsigned int sum = 0;
	unsigned int number_of_elements = 0;
	for (idx=0 ; idx<WIND_BUFFER_SIZE ; idx++) {
		// Take valid values only.
		if (wind_ctx.wind_direction_pourcent_buf[idx] != 0xFF) {
			sum += wind_ctx.wind_direction_pourcent_buf[idx];
			number_of_elements++;
		}
	}
	wind_ctx.wind_direction_pourcent_average = (sum / number_of_elements);
}

/*** WIND functions ***/

/* INIT WIND WIND VANE.
 * @param:	None.
 * @return:	None.
 */
void WIND_Init(void) {

	/* Init context */
	unsigned int idx = 0;
	// Wind speed.
	for (idx=0 ; idx<WIND_BUFFER_SIZE ; idx++) wind_ctx.wind_speed_count_buf[idx] = 0xFF;
	wind_ctx.wind_speed_count_buf_idx = 0;
	wind_ctx.wind_speed_count = 0;
	wind_ctx.wind_speed_kmh_average = 0;
	wind_ctx.wind_speed_kmh_peak = 0;
	// Wind direction.
	for (idx=0 ; idx<WIND_BUFFER_SIZE ; idx++) wind_ctx.wind_direction_pourcent_buf[idx] = 0xFF;
	wind_ctx.wind_direction_pourcent_buf_idx = 0;
	wind_ctx.wind_direction_pourcent = 0;

#ifndef WIND_DIRECTION_ANALOG
	/* Init phase shift timer and associated variables */
	TIM2_Init(TIM2_MODE_WIND, 0);
	wind_ctx.wind_speed_raw_counter = 0;
	wind_ctx.wind_direction_raw_counter = 0;
#endif
}

/* START CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StartContinuousMeasure(void) {
	// Reset second counter.
	wind_ctx.wind_seconds_count = 0;
#ifndef WIND_DIRECTION_ANALOG
	// Init TIM2 for wind operation.
	TIM2_Init(TIM2_MODE_WIND, 0);
	TIM2_Enable();
#endif
	// Enable required interrupts.
	NVIC_EnableInterrupt(IT_TIM21);
	NVIC_EnableInterrupt(IT_EXTI4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void WIND_StopContinuousMeasure(void) {
	// Disable required interrupts.
	NVIC_DisableInterrupt(IT_EXTI4_15);
	NVIC_DisableInterrupt(IT_TIM21);
#ifndef WIND_DIRECTION_ANALOG
	// Stop TIM2.
	TIM2_Stop();
	TIM2_Disable();
#endif
}

/* GET AVERAGE AND PEAK WIND SPEED VALUES SINCE LAST MEASUREMENT START.
 * @param average_wind_speed_kmh:	Pointer to byte that will contain average wind speed value in km/h.
 * @param peak_wind_speed_kmh:		Pointer to byte that will contain peak wind speed value in km/h.
 * @return:							None.
 */
void WIND_GetAveragePeakWindSpeed(unsigned char* average_wind_speed_kmh, unsigned char* peak_wind_speed_kmh) {
	WIND_ComputeAveragePeakWindSpeed();
	(*average_wind_speed_kmh) = wind_ctx.wind_speed_kmh_average;
	(*peak_wind_speed_kmh) = wind_ctx.wind_speed_kmh_peak;
}

/* GET AVERAGE AVERAGE WIND DIRECTION VALUE SINCE LAST MEASUREMENT START.
 * @param average_wind_directio_pourcent:	Pointer to byte that will contain average wind direction value in %.
 * @return:									None.
 */
void WIND_GetAverageWindDirection(unsigned char* average_wind_direction_pourcent) {
	WIND_ComputeAverageWindDirection();
	(*average_wind_direction_pourcent) = wind_ctx.wind_direction_pourcent_average;
}
