/*
 * fault.c
 *
 *  Created on: 7 nov. 2018
 *      Author: Ludo
 */

#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "pwr.h"
#include "types.h"

/* NON MASKABLE INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
#ifdef DEBUG
	// Blink LED.
	uint32_t k = 0;
	while(1) {
		GPIO_write(&GPIO_LED, 1);
		for (k=0 ; k<500000 ; k++);
		GPIO_write(&GPIO_LED, 0);
		for (k=0 ; k<500000 ; k++);
	}
#else
	// Trigger software reset.
	PWR_software_reset();
#endif
}

/* HARD FAULT INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
#ifdef DEBUG
	// Blink LED.
	uint32_t k = 0;
	while(1) {
		GPIO_write(&GPIO_LED, 1);
		for (k=0 ; k<100000 ; k++);
		GPIO_write(&GPIO_LED, 0);
		for (k=0 ; k<100000 ; k++);
	}
#else
	// Trigger software reset.
	PWR_software_reset();
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
