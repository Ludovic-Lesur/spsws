/*
 * max11136.c
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludovic
 */

#include "max11136.h"

#include "max11136_reg.h"
#include "rcc_reg.h"
#include "gpio_reg.h"
#include "spi.h"

/*** MAX11136 local functions ***/

/* MAX11136 REGISTER WRITE FUNCTION.
 * @param addr:		Register address (5 bits).
 * @param valie:	Value to write in register.
 * @return:			None.
 */
void MAX11136_WriteRegister(unsigned char addr, unsigned short value) {
	unsigned short spi_command = 0;

	/* Build SPI command */
	if (addr == MAX11136_REG_ADC_MODE_CONTROL) {
		// Data is 15-bits length.
		spi_command |= value & 0x00007FFF;
	}
	else {
		// Data is 11-bits length.
		spi_command |= (addr & 0x0000001F) << 11;
		spi_command |= (value & 0x000007FF);
	}

	/* Send command */
	GPIOB -> ODR &= ~(0b1 << 4); // Falling edge on CS pin.
	SPI_WriteShort(spi_command);
	GPIOB -> ODR |= (0b1 << 4); // Set CS pin.
}

/*** MAX11136 functions ***/

/* INIT MAX11136 ADC/
 * @param:	None.
 * @return:	None.
 */
void MAX11136_Init(void) {

	/* Init SPI peripheral */
	SPI_Init();

	/* Configure CS GPIO */
	RCC -> IOPENR |= (0b11 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8); // Configure PB4 as output.
	GPIOB -> ODR |= (0b1 << 4); // CS high (idle state).

	/* Configure EOC GPIO */
	GPIOB -> MODER &= ~(0b11 << 6); // Configure PB3 as input.
}

/* GET ADC CONVERSION RESULT ON ALL CHANNELS.
 * @param max11136_result:	Pointer to array that will contain ADC result on 12-bits.
 * @return:					None.
 */
void MAX11136_ConvertAllChannels(unsigned short* max11136_result) {

	/* Configure SPI */
	SPI_SetClockPolarity(1);

	/* Configure ADC */
	// External single ended: REFSEL='0' (allready done at POR).
	// Single-ended: unipolar_register=0 and bipolar_register=0 (allready done at POR).
	// Enable averaging: AVGON='1' and NAVG='00' (4 conversions).
	MAX11136_WriteRegister(MAX11136_REG_ADC_CONFIG, 0x0200);
	// Scan mode = standard internal: SCAN='0011'.
	// Perform conversion from AIN0 to AIN7: CHSEL='0111'.
	// Reset the FIFO: RESET='01'.
	// Auto shutdown: PM='01'.
	// Start conversion: SWCNV='1'.
	MAX11136_WriteRegister(MAX11136_REG_ADC_MODE_CONTROL, 0x1BA2);

	/* Wait for conversions to complete */
	while (((GPIOB -> IDR) & (0b1 << 3)) != 0); // Wait for EOC to be pulled low.

	/* Read results in FIFO */
	unsigned char idx = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) {
		GPIOB -> ODR &= ~(0b1 << 4); // Falling edge on CS pin.
		SPI_ReadShort(0x0000, &max11136_result[idx]);
		GPIOB -> ODR |= (0b1 << 4); // Set CS pin.
	}
}
