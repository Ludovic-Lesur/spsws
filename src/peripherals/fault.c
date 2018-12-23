/*
 * fault.c
 *
 *  Created on: 7 nov. 2018
 *      Author: Ludo
 */

void NMI_Handler(void) {
	// Trigger software reset.
	while (1);
}

void HardFault_Handler(void) {
	// Trigger software reset.
	while (1);
}
