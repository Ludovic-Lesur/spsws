/*
 * rain.c
 *
 *  Created on: 5 mai 2019
 *      Author: Ludovic
 */

#include "rain.h"

#include "at.h"
#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"

#if (defined CM || defined ATM)

/*** RAIN local macros ***/

// Pluviometry conversion ratio.
#define RAIN_EDGE_TO_UM		279
#define RAIN_MAX_VALUE_MM	0xFF

/*** RAIN local global variables ***/

volatile unsigned int rain_edge_count;

/*** RAIN functions ***/

/* INIT RAIN GAUGE.
 * @param:	None.
 * @return:	None.
 */
void RAIN_init(void) {
	// Init GPIOs and EXTI.
	GPIO_configure(&GPIO_DIO2, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_DIO2, EXTI_TRIGGER_FALLING_EDGE);
}

/* START CONTINUOUS RAIN MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void RAIN_start_continuous_measure(void) {
	// Enable required interrupt.
	EXTI_clear_all_flags();
	NVIC_enable_interrupt(NVIC_IT_EXTI_4_15);
}

/* STOP CONTINUOUS RAIN MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void RAIN_stop_continuous_measure(void) {
	// Disable required interrupt.
	NVIC_disable_interrupt(NVIC_IT_EXTI_4_15);
}

/* GET PLUVIOMETRY SINCE LAST MEASUREMENT START.
 * @param rain_pluviometry_mm:	Pointer to char that will contain pluviometry in mm.
 * @return:						None.
 */
void RAIN_get_pluviometry(unsigned char* rain_pluviometry_mm) {
	// Convert edge count to mm of rain.
	unsigned int rain_um = (rain_edge_count * RAIN_EDGE_TO_UM);
	unsigned int rain_mm = (rain_um / 1000);
	// Rounding operation.
	unsigned int remainder = rain_um - (rain_mm * 1000);
	if (remainder >= 500) {
		rain_mm++;
	}
	// Clip value.
	if (rain_mm > RAIN_MAX_VALUE_MM) {
		(*rain_pluviometry_mm) = RAIN_MAX_VALUE_MM;
	}
	else {
		(*rain_pluviometry_mm) = (unsigned char) rain_mm;
	}
}

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
