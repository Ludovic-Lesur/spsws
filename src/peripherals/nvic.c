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

/*** NVIC local global variables ***/

extern uint32_t __Vectors;

/*** NVIC functions ***/

/* INIT VECTOR TABLE ADDRESS.
 * @param:	None.
 * @return:	None.
 */
void NVIC_init(void) {
	SCB -> VTOR = (uint32_t) &__Vectors;
}

/* ENABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (use enum defined in 'nvic.h').
 * @return: 		None.
 */
void NVIC_enable_interrupt(NVIC_interrupt_t it_num) {
	NVIC -> ISER = (0b1 << (it_num & 0x1F));
}

/* DISABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (use enum defined in 'nvic.h').
 * @return:			None.
 */
void NVIC_disable_interrupt(NVIC_interrupt_t it_num) {
	NVIC -> ICER = (0b1 << (it_num & 0x1F));
}

/* SET THE PRIORITY OF AN INTERRUPT LINE.
 * @param it_num:	Interrupt number (use enum defined in 'nvic.h').
 * @param priority:	Interrupt priority (0 to 3).
 */
void NVIC_set_priority(NVIC_interrupt_t it_num, uint8_t priority) {
	// Check parameter.
	if (priority > NVIC_PRIORITY_MIN) return;
	// Reset bits.
	NVIC -> IPR[(it_num >> 2)] &= ~(0xFF << (8 * (it_num % 4)));
	// Set priority.
	NVIC -> IPR[(it_num >> 2)] |= ((priority << 6) << (8 * (it_num % 4)));
}
