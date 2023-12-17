/*
 * fault.c
 *
 *  Created on: 7 nov. 2018
 *      Author: Ludo
 */

#include "pwr.h"

/*******************************************************************/
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}
