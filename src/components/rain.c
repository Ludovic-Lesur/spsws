/*
 * rain.c
 *
 *  Created on: 5 mai 2019
 *      Author: Ludo
 */

#include "rain.h"

#include "at.h"
#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "types.h"

#if (defined CM || defined ATM)

/*** RAIN local macros ***/

// Pluviometry conversion ratio.
#define RAIN_EDGE_TO_UM				279
#define RAIN_MAX_VALUE_MM			0xFF
#ifdef FLOOD_DETECTION
// Flood alarm levels mapping.
#define RAIN_GPIO_FLOOD_LEVEL_1		GPIO_DIO1
#define RAIN_GPIO_FLOOD_LEVEL_2		GPIO_DIO3
#define RAIN_GPIO_FLOOD_LEVEL_3		GPIO_DIO4
#endif

/*** RAIN local global variables ***/

volatile uint32_t rain_edge_count;

/*** RAIN functions ***/

/* INIT RAIN GAUGE.
 * @param:	None.
 * @return:	None.
 */
void RAIN_init(void) {
	// Init GPIOs and EXTI.
	GPIO_configure(&GPIO_DIO2, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_DIO2, EXTI_TRIGGER_FALLING_EDGE);
#ifdef FLOOD_DETECTION
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_1, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_2, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&RAIN_GPIO_FLOOD_LEVEL_3, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/* START CONTINUOUS RAIN MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void RAIN_start_continuous_measure(void) {
	// Enable required interrupt.
	EXTI_clear_all_flags();
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
}

/* STOP CONTINUOUS RAIN MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void RAIN_stop_continuous_measure(void) {
	// Disable required interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
}

/* GET PLUVIOMETRY SINCE LAST MEASUREMENT START.
 * @param pluviometry_mm:	Pointer to 8-bits value that will contain pluviometry in mm.
 * @return status:			Function execution status.
 */
RAIN_status_t RAIN_get_pluviometry(uint8_t* pluviometry_mm) {
	// Local variables.
	RAIN_status_t status = RAIN_SUCCESS;
	uint32_t rain_um = 0;
	uint32_t rain_mm = 0;
	uint32_t remainder = 0;
	// Check parameter.
	if (pluviometry_mm == NULL) {
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
	if (rain_mm > RAIN_MAX_VALUE_MM) {
		(*pluviometry_mm) = RAIN_MAX_VALUE_MM;
	}
	else {
		(*pluviometry_mm) = (uint8_t) rain_mm;
	}
errors:
	return status;
}

#ifdef FLOOD_DETECTION
/* GET CURRENT FLOOD LEVEL.
 * @param flood_level:	Pointer to 8-bits value that will current flood level.
 * @return status:		Function execution status.
 */
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

/* RESET RAIN MEASUREMENT DATA.
 * @param:	None.
 * @return:	None.
 */
void RAIN_reset_data(void) {
	// Reset edge count.
	rain_edge_count = 0;
}

/* FUNCTION CALLED BY EXTI INTERRUPT HANDLER WHEN AN EDGE IS DETECTED ON THE RAIN SIGNAL.
 * @param:	None.
 * @return:	None.
 */
void RAIN_edge_callback(void) {
	// Increment edge count.
	rain_edge_count++;
#ifdef ATM
	AT_print_rain(rain_edge_count);
#endif
}

#endif
