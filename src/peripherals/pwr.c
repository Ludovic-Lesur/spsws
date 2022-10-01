/*
 * pwr.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "pwr.h"

#include "exti_reg.h"
#include "flash_reg.h"
#include "nvic_reg.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "scb_reg.h"

/*** PWR functions ***/

/* INIT PWR INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void PWR_init(void) {
	// Enable power interface clock.
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	// Unlock back-up registers (DBP bit).
	PWR -> CR |= (0b1 << 8);
	// Power memories down when entering sleep mode.
	FLASH -> ACR |= (0b1 << 3); // SLEEP_PD='1'.
	// Use HSI clock when waking-up from stop mode.
	RCC -> CFGR |= (0b1 << 15);
	// Switch internal voltage reference off in low power mode.
	PWR -> CR |= (0b1 << 9); // ULP='1'.
	// Ignore internal voltage reference startup time on wake-up.
	PWR -> CR |= (0b1 << 10); // FWU='1'.
	// Never return in low power sleep mode after wake-up.
	SCB -> SCR &= ~(0b1 << 1); // SLEEPONEXIT='0'.
}

/* FUNCTION TO ENTER SLEEP MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_enter_sleep_mode(void) {
	// Regulator in normal mode.
	PWR -> CR &= ~(0b1 << 0); // LPSDSR='0'.
	// Enter low power sleep mode.
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}

/* FUNCTION TO ENTER STOP MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_enter_stop_mode(void) {
	// Regulator in low power mode.
	PWR -> CR |= (0b1 << 0); // LPSDSR='1'.
	// Clear WUF flag.
	PWR -> CR |= (0b1 << 2); // CWUF='1'.
	// Enter stop mode when CPU enters deepsleep.
	PWR -> CR &= ~(0b1 << 1); // PDDS='0'.
	// Clear all EXTI, RTC and peripherals interrupt pending bits.
	RCC -> CICR |= 0x000001BF;
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
	RTC -> ISR &= 0xFFFE035F; // Reset wake-up, tamper and timestamp flags.
	// Note: RTC alarms flags are not cleared because alarm A flag is read statically after wake-up in the main.
	// If the alarm A or B flag is set when entering stop mode, the execution will be ignored but the main will directly treat and clear the flag.
	// The MCU will enter stop on the second loop.
	NVIC -> ICPR = 0xFFFFFFFF; // CLEARPENDx='1'.
	// Enter stop mode.
	SCB -> SCR |= (0b1 << 2); // SLEEPDEEP='1'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}
