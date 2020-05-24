/*
 * pwr.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "pwr.h"

#include "exti_reg.h"
#include "flash_reg.h"
#include "mapping.h"
#include "nvic_reg.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "rcc.h"
#include "rtc_reg.h"
#include "scb_reg.h"

/*** PWR functions ***/

/* FUNCTION TO ENTER LOW POWER SLEEP MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_EnterLowPowerSleepMode(void) {

	/* Enable power interface clock */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.

	/* Power memories down when entering sleep mode */
	FLASH -> ACR |= (0b1 << 3); // SLEEP_PD='1'.

	/* Regulator in low power mode */
	PWR -> CR |= (0b1 << 0); // LPSDSR='1'.

	/* Switch to 65kHz MSI clock */
	RCC_SwitchToMsi();
	RCC_DisableGpio();

	/* Enter low power sleep mode */
	SCB -> SCR &= ~(0b1 << 1); // Do not return in low power sleep mode after wake-up (SLEEPONEXIT='0').
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}

/* FUNCTION TO ENTER STOP MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_EnterStopMode(void) {
	// Enable power interface clock.
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	// Use MSI clock when waking-up from stop mode.
	RCC -> CFGR &= ~(0b1 << 15);
	// Clear WUF flag.
	PWR -> CR |= (0b1 << 2); // CWUF='1'.
	// Switch internal voltage reference off in low power mode.
	PWR -> CR |= (0b1 << 9); // ULP='1'.
	// Enter stop mode when CPU enters deepsleep.
	PWR -> CR &= ~(0b1 << 1); // PDDS='0'.
	// Clear all EXTI line, RTC an peripherals interrupt pending bits.
	RCC -> CICR |= 0x000001BF;
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
	RTC -> ISR &= 0xFFFF005F; // Reset alarms, wake-up, tamper and timestamp flags.
	NVIC -> ICPR = 0xFFFFFFFF; // CLEARPENDx='1'.
	// Enter standby mode.
	SCB -> SCR &= ~(0b1 << 1); // Do not return in stop mode after wake-up (SLEEPONEXIT='0').
	SCB -> SCR |= (0b1 << 2); // SLEEPDEEP='1'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}

/* FUNCTION TO ENTER STANDBY MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_EnterStandbyMode(void) {

	/* Enable power interface clock */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.

#ifdef USE_HWT
	/* Configure WKUP pin */
	PWR -> CSR |= (0b1 << 8); // Use WKUP1 pin (PA0) rising edge to exit standby mode (EWUP1='1').
#endif

	/* Clear WUF flag */
	PWR -> CR |= (0b1 << 2); // CWUF='1'.

	/* Switch internal voltage reference off in low power mode */
	PWR -> CR |= (0b1 << 9); // ULP='1'.

	/* Enter standy mode when CPU enters deepsleep */
	PWR -> CR |= (0b1 << 1); // PDDS='1'.

	/* Enter standby mode */
	SCB -> SCR &= ~(0b1 << 1); // Do not return in standby after wake-up (SLEEPONEXIT='0').
	SCB -> SCR |= (0b1 << 2); // SLEEPDEEP='1'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}
