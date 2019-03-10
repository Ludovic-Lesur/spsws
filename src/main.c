/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Peripherals.
#include "adc.h"
#include "aes.h"
#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "i2c.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "spi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
// Components.
#include "dps310.h"
#include "max11136.h"
#include "max5495.h"
#include "neom8n.h"
#include "sht3x.h"
#include "si1133.h"
#include "sky13317.h"
#include "sx1232.h"
#include "sigfox_types.h"
#include "wind.h"
// Applicative.
#include "at.h"
#include "geoloc.h"
#ifdef IM_HWT
#include "hwt.h"
#endif
#include "mode.h"
#include "sigfox_api.h"

/*** SPSWS structures ***/

typedef enum {
	SPSWS_STATUS_BYTE_STATION_MODE_BIT_IDX,
	SPSWS_STATUS_BYTE_TIME_REFERENCE_BIT_IDX,
	SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_FIRST_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_RTC_DAILY_CALIBRATION_BIT_IDX,
	SPSWS_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX,
} SPSWS_StatusByteBitIndex;

typedef struct {
	unsigned char spsws_status_byte;
} SPSWS_Context;

/*** SPSWS global variables ***/

static SPSWS_Context spsws_ctx;

/*** SPSWS main function ***/

#ifdef IM_RTC
int main (void) {
	return 0;
}
#endif

#ifdef CM_RTC
int main (void) {
	return 0;
}
#endif

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init context */
	spsws_ctx.spsws_status_byte = 0;

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();

	/* Init clocks */
	RCC_Init();
	// High speed oscillator.
	spsws_ctx.spsws_status_byte |= (RCC_SwitchToTcxo16MHz() << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
	if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
		while (RCC_SwitchToInternal16MHz() == 0);
	}
	// Low speed oscillator.
	spsws_ctx.spsws_status_byte |= (RCC_SwitchToQuartz32kHz() << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX);
	if ((spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX)) == 0) {
		while (RCC_SwitchToInternal32kHz() == 0);
	}

	/* Init peripherals */
	// Real time clock.
	RTC_Init(spsws_ctx.spsws_status_byte & (0b1 << SPSWS_STATUS_BYTE_RTC_CLOCK_SOURCE_BIT_IDX));
	// External interrupts.
	EXTI_Init();
	// Timers.
	TIM21_Init();
	TIM21_Enable();
	TIM22_Init();
	TIM22_Enable();
	LPTIM1_Init();
	// DMA.
	DMA1_Init();
	// Analog.
	ADC1_Init();
	// Communication interfaces.
	LPUART1_Init();
#ifdef HW1_0
	USART2_Init();
#endif
#ifdef HW2_0
	USART1_Init();
#endif
	I2C1_Init();
	SPI1_Init();
#ifdef HW2_0
	SPI2_Init();
#endif
	// Hardware AES.
	AES_Init();

	/* Init components */
	SX1232_Init();
	SKY13317_Init();
	NEOM8N_Init();
	MAX11136_Init();
	WIND_Init();
	SHT3X_Init();
	DPS310_Init();
	SI1133_Init();

	/* Init applicative layers */
	AT_Init();

	/* Main loop */
	while (1) {
		AT_Task();
	}

	return 0;
}
#endif
