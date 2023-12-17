/*
 * nvic.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "nvic.h"

#include "nvic_reg.h"
#include "scb_reg.h"
#include "types.h"

/*** NVIC local macros ***/

#define NVIC_PRIORITY_MIN	3

/*** NVIC local global variables ***/

extern uint32_t __Vectors;

/*** NVIC local functions ***/

/*******************************************************************/
static void _NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority) {
	// Local variables.
	uint8_t local_priority = priority;
	// Clamp parameter.
	if (local_priority > NVIC_PRIORITY_MIN) {
		local_priority = NVIC_PRIORITY_MIN;
	}
	// Reset bits.
	NVIC -> IPR[(irq_index >> 2)] &= ~(0xFF << (8 * (irq_index % 4)));
	// Set priority.
	NVIC -> IPR[(irq_index >> 2)] |= ((local_priority << 6) << (8 * (irq_index % 4)));
}

/*** NVIC functions ***/

/*******************************************************************/
void NVIC_init(void) {
	// Init vector table address.
	SCB -> VTOR = (uint32_t) &__Vectors;
}

/*******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority) {
	// Set priority.
	_NVIC_set_priority(irq_index, priority);
	// Enable interrupt.
	NVIC -> ISER = (0b1 << (irq_index & 0x1F));
}

/*******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index) {
	// Disable interrupt.
	NVIC -> ICER = (0b1 << (irq_index & 0x1F));
}
