/*
 * max11136.c
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#include "max11136.h"

#include "gpio.h"
#include "mapping.h"
#include "max11136_reg.h"
#include "spi.h"

/*** MAX11136 local macros ***/

#define MAX11136_NUMBER_OF_CHANNELS		8
#define MAX11136_CONVERSION_LOOPS		3

/*** MAX11136 local structures ***/

typedef struct {
	unsigned short max11136_result_12bits[MAX11136_NUMBER_OF_CHANNELS];
	unsigned char max11136_status; // Bit 'i' indicates if a result was successfully retrieved for channel 'i'.
} MAX11136_Context;

/*** MAX11136 local global variables ***/

static MAX11136_Context max11136_ctx;

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
	GPIO_Write(GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
#ifdef HW1_0
	SPI1_WriteShort(spi_command);
#endif
#ifdef HW2_0
	SPI2_WriteShort(spi_command);
#endif
	GPIO_Write(GPIO_MAX11136_CS, 1); // Set CS pin.
}

/* PERFORM ADC CONVERSION ON ALL CHANNELS.
 * @param:					None.
 * @return max11136_status:	Conversion result status (see context).
 */
void MAX11136_ConvertAllChannels12Bits(void) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(1);
#endif

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
	MAX11136_WriteRegister(MAX11136_REG_ADC_MODE_CONTROL, 0x1BAA);

	/* Wait for conversions to complete */
#ifdef USE_MAX11136_EOC
	while (GPIO_Read(GPIO_MAX11136_EOC) != 0); // Wait for EOC to be pulled low.
#else
	// TBD: configure CS as input.
#endif

	/* Read results in FIFO */
	unsigned short max11136_dout = 0;
	unsigned char channel = 0;
	unsigned char channel_idx = 0;
	// Wait for all channels to be read or timeout (TBD).
	for (channel_idx=0 ; channel_idx<MAX11136_NUMBER_OF_CHANNELS ; channel_idx++) {
		// Get data from SPI.
		GPIO_Write(GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
#ifdef HW1_0
		SPI1_ReadShort(0x0000, &max11136_dout);
#endif
#ifdef HW2_0
		SPI2_ReadShort(0x0000, &max11136_dout);
#endif
		GPIO_Write(GPIO_MAX11136_CS, 1); // Set CS pin.
		// Parse result = 'CH4 CH2 CH1 CH0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0'.
		channel = (max11136_dout & 0xF000) >> 12;
		if (channel < MAX11136_NUMBER_OF_CHANNELS) {
			// Fill data.
			max11136_ctx.max11136_result_12bits[channel] = max11136_dout & 0x0FFF;
			// Update status.
			max11136_ctx.max11136_status |= (0b1 << channel);
		}
	}
}

/*** MAX11136 functions ***/

/* INIT MAX11136 ADC/
 * @param:	None.
 * @return:	None.
 */
void MAX11136_Init(void) {

	/* Init context */
	unsigned char idx = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) max11136_ctx.max11136_result_12bits[idx] = 0;
	max11136_ctx.max11136_status = 0;

	/* Configure EOC GPIO */
#ifdef USE_MAX11136_EOC
	GPIO_Configure(GPIO_MAX11136_EOC, Input, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif
}

/* PERFORM ADC CONVERSION ON ALL CHANNELS.
 * @param:					None.
 * @return max11136_status:	Conversion result status (see context).
 */
unsigned char MAX11136_PerformMeasurements(void) {

	/* Reset results */
	unsigned char idx = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) max11136_ctx.max11136_result_12bits[idx] = 0;
	max11136_ctx.max11136_status = 0;

	/* Perform conversion until all channels are successfully retrieved or maximum number of loops is reached */
#ifdef HW1_0
	SPI1_Enable();
#endif
#ifdef HW2_0
	SPI2_Enable();
#endif
	idx = 0;
	while ((max11136_ctx.max11136_status != 0xFF) && (idx < MAX11136_CONVERSION_LOOPS)) {
		MAX11136_ConvertAllChannels12Bits();
		idx++;
	}
#ifdef HW1_0
	SPI1_Disable();
#endif
#ifdef HW2_0
	SPI2_Disable();
#endif

	/* Return status */
	return max11136_ctx.max11136_status;
}

/* GET CHANNEL RESULT.
 * @param channel:					ADC channel to get.
 * @param channel_voltage_12bits:	Pointer to int that will contain the channel result on 12 bits.
 * @return:							None.
 */
void MAX11136_GetChannel(unsigned char channel, unsigned int* channel_result_12bits) {
	// Check parameter.
	if (channel < MAX11136_NUMBER_OF_CHANNELS) {
		(*channel_result_12bits) = max11136_ctx.max11136_result_12bits[channel];
	}
}
