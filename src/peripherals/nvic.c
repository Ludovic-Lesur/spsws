/*
 * nvic.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "nvic.h"
#include "nvic_reg.h"

/*** NVIC functions ***/

/* ENABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (0 to 29, use the InterruptVector enum defined in 'nvic_reg.h').
 * @return: 		None.
 */
void NVIC_EnableInterrupt(InterruptVector it_num) {
	NVIC -> ISER = (0b1 << (it_num & 0x0000001F));
}

/* DISABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (0 to 29, use the InterruptVector enum defined in 'nvic_reg.h').
 * @return:			None.
 */
void NVIC_DisableInterrupt(InterruptVector it_num) {
	NVIC -> ICER = (0b1 << (it_num & 0x0000001F));
}

/* SET THE PRIORITY OF AN INTERRUPT LINE.
 * @param it_num:	Interrupt number (0 to 97, use the InterruptVector enum defined in 'nvic_reg.h').
 * @param priority:	The priority to set (between 0 and 255).
 */
void NVIC_SetPriority(InterruptVector it_num, unsigned char priority) {
	NVIC -> IPR[it_num] = priority;
}

/* @NOTE:
 * To add an interrupt from a given peripheral:
 * 		1) Configure and enable interrupts in the proper(s) peripheral register(s).
 * 		2) Add 'NVIC_EnableInterrupt(<IT>);' where <IT> is the interrupt number in NVIC (specified in 'InterruptVector' enumeration, see nvic.h).
 * 		3) startup_stm32.s, g_pfnVectors: add '.word <handlerFunction>' at the line corresponding to <IT>.
 * 		4) startup_stm32.s, g_pfnVectors: add '.weak <handlerFunction>
											   .thumb_set <handlerFunction>,Default_Handler'.
		5) Implement 'void <handlerFunction>(void)' in the peripheral source file.
 */
