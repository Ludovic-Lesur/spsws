/*
 * fault.c
 *
 *  Created on: 7 nov. 2018
 *      Author: Ludo
 */

#include "scb_reg.h"

/* NON MASKABLE INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void NMI_Handler(void) {
	// Trigger software reset.
	SCB -> AIRCR |= (0b1 << 2);
}

/* HARD FAULT INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void HardFault_Handler(void) {
	// Trigger software reset.
	SCB -> AIRCR |= (0b1 << 2);
}
