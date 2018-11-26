/*
 * ultimeter.c
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludovic
 */

#include "ultimeter.h"

#include "nvic.h"
#include "tim.h"
#include "usart.h"

/*** Ultimeter local macros ***/

#define ULTIMETER_BUFFER_SIZE	(3600 / ULTIMETER_MEASUREMENT_PERIOD_SECONDS) // 1 hour buffer.

/*** Ultimeter local structures ***/

typedef struct {
	// Wind speed.
	unsigned char ultimeter_wind_speed_buf[ULTIMETER_BUFFER_SIZE]; // Stores revolution counter for each measurement period.
	unsigned int ultimeter_wind_speed_buf_idx;
	unsigned char ultimeter_wind_speed_count; // Current revolution counter value.
	unsigned char ultimeter_wind_speed_average_kmh; // Average wind speed (km/h).
	unsigned char ultimeter_wind_speed_peak_kmh; // Peak wind speed (km/h).
	// Wind direction.
	unsigned char ultimeter_wind_direction_buf[ULTIMETER_BUFFER_SIZE]; // Stores wind direction (%) for each measurement period.
	unsigned int ultimeter_wind_direction_buf_idx;
	unsigned char ultimeter_wind_direction; // Current wind direction value (%).
	unsigned char ultimeter_wind_direction_average_pourcent; // Average wind direction (°).
} ULTIMETER_Context;

/*** Ultimeter local global variables ***/

volatile ULTIMETER_Context ultimeter_ctx;

/*** Ultimeter local functions ***/

void ULTIMETER_ComputeAverageWindSpeed(void) {
	/* Compute average speed counts in buffer */
	unsigned int idx = 0;
	unsigned int sum = 0;
	unsigned int number_of_elements = 0;
	for (idx=0 ; idx<ULTIMETER_BUFFER_SIZE ; idx++) {
		// Take valid values only.
		if (ultimeter_ctx.ultimeter_wind_speed_buf[idx] != 0xFF) {
			sum += ultimeter_ctx.ultimeter_wind_speed_buf[idx];
			number_of_elements++;
		}
	}
	unsigned int mean_wind_speed_count = (sum / number_of_elements);

	/* Convert value to km/h */
	ultimeter_ctx.ultimeter_wind_speed_average_kmh = mean_wind_speed_count;
}

void ULTIMETER_ComputePeakWindSpeed(void) {
	/* Search maximum speed counts in buffer */
	/* Compute average speed counts in buffer */
	unsigned int idx = 0;
	unsigned int max = 0;
	for (idx=0 ; idx<ULTIMETER_BUFFER_SIZE ; idx++) {
		// Take valid values only.
		if ((ultimeter_ctx.ultimeter_wind_speed_buf[idx] != 0xFF) && (ultimeter_ctx.ultimeter_wind_speed_buf[idx] > max)) {
			max = ultimeter_ctx.ultimeter_wind_speed_buf[idx];
		}
	}

	/* Convert value to km/h */
	ultimeter_ctx.ultimeter_wind_speed_peak_kmh = max;
}

void ULTIMETER_ComputeAverageWindDirection(void) {
	/* Compute average speed counts in buffer */
	unsigned int idx = 0;
	unsigned int sum = 0;
	unsigned int number_of_elements = 0;
	for (idx=0 ; idx<ULTIMETER_BUFFER_SIZE ; idx++) {
		// Take valid values only.
		if (ultimeter_ctx.ultimeter_wind_direction_buf[idx] != 0xFF) {
			sum += ultimeter_ctx.ultimeter_wind_direction_buf[idx];
			number_of_elements++;
		}
	}
	ultimeter_ctx.ultimeter_wind_direction_average_pourcent = (sum / number_of_elements);
}

/*** Ultimeter functions ***/

/* INIT ULTIMETER WIND VANE.
 * @param:	None.
 * @return:	None.
 */
void ULTIMETER_Init(void) {

	/* Init context */
	unsigned int idx = 0;
	// Wind speed.
	for (idx=0 ; idx<ULTIMETER_BUFFER_SIZE ; idx++) ultimeter_ctx.ultimeter_wind_speed_buf[idx] = 0xFF;
	ultimeter_ctx.ultimeter_wind_speed_buf_idx = 0;
	ultimeter_ctx.ultimeter_wind_speed_count = 0;
	ultimeter_ctx.ultimeter_wind_speed_average_kmh = 0;
	ultimeter_ctx.ultimeter_wind_speed_peak_kmh = 0;
	// Wind direction.
	for (idx=0 ; idx<ULTIMETER_BUFFER_SIZE ; idx++) ultimeter_ctx.ultimeter_wind_direction_buf[idx] = 0xFF;
	ultimeter_ctx.ultimeter_wind_direction_buf_idx = 0;
	ultimeter_ctx.ultimeter_wind_direction = 0;

	/* Init phase shift timer */
	TIM2_Init();
}

/* INCREMENT CURRENT WIND SPEED COUNT.
 * @param:	None.
 * @return:	None.
 */
void ULTIMETER_IncrementWindSpeedCount(void) {
	// Update count.
	ultimeter_ctx.ultimeter_wind_speed_count++;
}

/* UPDATE CURRENT WIND DIRECTION.
 * @param new_wind_direction_pourcent:	New value of wind direction (%).
 * @return:								None;
 */
void ULTIMETER_UpdateWindDirection(unsigned char new_wind_direction_pourcent) {
	// Update value.
	ultimeter_ctx.ultimeter_wind_direction = new_wind_direction_pourcent;
}

/* STORE CURRENT MEASUREMENTS IN BUFFERS.
 * @param:	None.
 * @return:	None.
 */
void ULTIMETER_StoreMeasurements(void) {

	/* Wind speed */
	// Store current counter.
	ultimeter_ctx.ultimeter_wind_speed_buf[ultimeter_ctx.ultimeter_wind_speed_buf_idx] = ultimeter_ctx.ultimeter_wind_speed_count;
	//USART_SendString("Speed count = ");
	//USART_SendValue(ultimeter_ctx.ultimeter_wind_speed_count, USART_Decimal);
	// Reset counter.
	ultimeter_ctx.ultimeter_wind_speed_count = 0;
	// Increment index and manage roll-over.
	ultimeter_ctx.ultimeter_wind_speed_buf_idx++;
	if (ultimeter_ctx.ultimeter_wind_speed_buf_idx == ULTIMETER_BUFFER_SIZE) {
		ultimeter_ctx.ultimeter_wind_speed_buf_idx = 0;
	}

	/* Wind direction */
	// Store current direction.
	ultimeter_ctx.ultimeter_wind_direction_buf[ultimeter_ctx.ultimeter_wind_direction_buf_idx] = ultimeter_ctx.ultimeter_wind_direction;
	//USART_SendString(" Direction = ");
	//USART_SendValue(ultimeter_ctx.ultimeter_wind_direction, USART_Decimal);
	//USART_SendString(" %\n");
	// Increment index and manage roll-over.
	ultimeter_ctx.ultimeter_wind_direction_buf_idx++;
	if (ultimeter_ctx.ultimeter_wind_direction_buf_idx == ULTIMETER_BUFFER_SIZE) {
		ultimeter_ctx.ultimeter_wind_direction_buf_idx = 0;
	}
}

/* START CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void ULTIMETER_StartContinuousMeasure(void) {
	// Enable required interrupts.
	NVIC_EnableInterrupt(IT_TIM21);
	NVIC_EnableInterrupt(IT_EXTI4_15);
}

/* STOP CONTINUOUS WIND MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void ULTIMETER_StopContinuousMeasure(void) {
	// Disable required interrupts.
	NVIC_DisableInterrupt(IT_EXTI4_15);
	NVIC_DisableInterrupt(IT_TIM21);
	// Stop TIM2.
	TIM2_Stop();
}

void ULTIMETER_GetAverageWindSpeed(unsigned char* average_wind_speed_kmh) {
	ULTIMETER_ComputeAverageWindSpeed();
	(*average_wind_speed_kmh) = ultimeter_ctx.ultimeter_wind_speed_average_kmh;
}

void ULTIMETER_GetPeakWindSpeed(unsigned char* peak_wind_speed_kmh) {
	ULTIMETER_ComputePeakWindSpeed();
	(*peak_wind_speed_kmh) = ultimeter_ctx.ultimeter_wind_speed_peak_kmh;
}

void ULTIMETER_GetAverageWindDirection(unsigned char* average_wind_direction_pourcent) {
	ULTIMETER_ComputeAverageWindDirection();
	(*average_wind_direction_pourcent) = ultimeter_ctx.ultimeter_wind_direction_average_pourcent;
}
