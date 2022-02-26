/*
 * fault.c
 *
 *  Created on: 7 nov. 2018
 *      Author: Ludo
 */

#include "scb_reg.h"
#include "gpio.h"
#include "mapping.h"
#include "mode.h"

/* NON MASKABLE INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
#ifdef DEBUG
	// Blink LED.
	unsigned int k = 0;
	while(1) {
		GPIO_write(&GPIO_LED, 1);
		for (k=0 ; k<500000 ; k++);
		GPIO_write(&GPIO_LED, 0);
		for (k=0 ; k<500000 ; k++);
	}
#else
	// Trigger software reset.
	SCB -> AIRCR = 0x05FA0000 | ((SCB -> AIRCR) & 0x0000FFFF) | (0b1 << 2);
#endif
}

/* HARD FAULT INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
#ifdef DEBUG
	// Blink LED.
	unsigned int k = 0;
	while(1) {
		GPIO_write(&GPIO_LED, 1);
		for (k=0 ; k<100000 ; k++);
		GPIO_write(&GPIO_LED, 0);
		for (k=0 ; k<100000 ; k++);
	}
#else
	// Trigger software reset.
	SCB -> AIRCR = 0x05FA0000 | ((SCB -> AIRCR) & 0x0000FFFF) | (0b1 << 2);
#endif
}

/* SVC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) SVC_Handler(void) {
	// TBD.
}

/* PENDING SV INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) PendSV_Handler(void) {
	// TBD.
}

/* SYSTEM TICK INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) SysTick_Handler(void) {
	// TBD.
}
