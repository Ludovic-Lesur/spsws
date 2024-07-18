/*
 * rain.c
 *
 *  Created on: 5 may 2019
 *      Author: Ludo
 */

#include "rain.h"

#include "at.h"
#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "types.h"

/*** RAIN local macros ***/

#ifdef SPSWS_RAIN_MEASUREMENT
// DIOs mapping.
#define RAIN_GPIO_DETECT			GPIO_DIO2
// Rainfall conversion ratio.
#define RAIN_EDGE_TO_UM				279
#define RAIN_MAX_VALUE_MM			0xFF
#endif
#ifdef SPSWS_FLOOD_MEASUREMENT
// DIOs mapping.
#define RAIN_GPIO_FLOOD_LEVEL_1		GPIO_DIO1
#define RAIN_GPIO_FLOOD_LEVEL_2		GPIO_DIO3
#define RAIN_GPIO_FLOOD_LEVEL_3		GPIO_DIO4
#endif

/*** RAIN local global variables ***/

#if (defined SPSWS_RAIN_MEASUREMENT)
volatile uint32_t rain_edge_count = 0;
#endif

/*** RAIN local functions ***/

#if (defined SPSWS_RAIN_MEASUREMENT)
/*******************************************************************/
static void _RAIN_edge_callback(void) {
	// Increment edge count.
	rain_edge_count++;
#ifdef ATM
	AT_print_rainfall(rain_edge_count);
#endif
}
#endif

/*** RAIN functions ***/

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*******************************************************************/
void RAIN_init(void) {
	// Init GPIOs and EXTI.
#ifdef SPSWS_RAIN_MEASUREMENT
	GPIO_configure(&RAIN_GPIO_DETECT, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&RAIN_GPIO_DETECT, EXTI_TRIGGER_FALLING_EDGE, &_RAIN_edge_callback);
#endif
#ifdef SPSWS_FLOOD_MEASUREMENT
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_1, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_2, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_3, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}
#endif

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*******************************************************************/
void RAIN_start_continuous_measure(void) {
	// Enable interrupt.
	EXTI_clear_flag((EXTI_line_t) RAIN_GPIO_DETECT.pin);
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_4_15, NVIC_INTERRUPT_EXTI_4_15);
}
#endif

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*******************************************************************/
void RAIN_stop_continuous_measure(void) {
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
}
#endif

#ifdef SPSWS_RAIN_MEASUREMENT
/*******************************************************************/
void RAIN_reset_rainfall(void) {
	// Reset edge count.
	rain_edge_count = 0;
}
#endif

#ifdef SPSWS_RAIN_MEASUREMENT
/*******************************************************************/
RAIN_status_t RAIN_get_rainfall(uint8_t* rainfall_mm) {
	// Local variables.
	RAIN_status_t status = RAIN_SUCCESS;
	uint32_t rain_um = 0;
	uint32_t rain_mm = 0;
	uint32_t remainder = 0;
	// Check parameter.
	if (rainfall_mm == NULL) {
		status = RAIN_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Convert edge count to mm of rain.
	rain_um = (rain_edge_count * RAIN_EDGE_TO_UM);
	rain_mm = (rain_um / 1000);
	// Rounding operation.
	remainder = rain_um - (rain_mm * 1000);
	if (remainder >= 500) {
		rain_mm++;
	}
	// Clip value.
	(*rainfall_mm) = (rain_mm > RAIN_MAX_VALUE_MM) ? RAIN_MAX_VALUE_MM : ((uint8_t) rain_mm);
errors:
	return status;
}
#endif

#ifdef SPSWS_FLOOD_MEASUREMENT
/*******************************************************************/
RAIN_status_t RAIN_get_flood_level(uint8_t* flood_level) {
	// Local variables.
	RAIN_status_t status = RAIN_SUCCESS;
	// Check parameter.
	if (flood_level == NULL) {
		status = RAIN_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read inputs.
	if (GPIO_read(&RAIN_GPIO_FLOOD_LEVEL_3) == 0) {
		(*flood_level) = 3;
	}
	else if (GPIO_read(&RAIN_GPIO_FLOOD_LEVEL_2) == 0) {
		(*flood_level) = 2;
	}
	else if (GPIO_read(&RAIN_GPIO_FLOOD_LEVEL_1) == 0) {
		(*flood_level) = 1;
	}
	else {
		(*flood_level) = 0;
	}
errors:
	return status;
}
#endif
