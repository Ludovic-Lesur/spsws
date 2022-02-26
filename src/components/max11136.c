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

#define MAX11136_CONVERSION_LOOPS				3
#define MAX11136_TIMEOUT_COUNT					1000000

/*** MAX11136 local structures ***/

typedef struct {
	unsigned short result_12bits[MAX11136_NUMBER_OF_CHANNELS];
	unsigned char status; // Bit 'i' indicates if a result was successfully retrieved for channel 'i'.
} MAX11136_context_t;

/*** MAX11136 local global variables ***/

static MAX11136_context_t max11136_ctx;

/*** MAX11136 local functions ***/

/* MAX11136 REGISTER WRITE FUNCTION.
 * @param addr:		Register address (5 bits).
 * @param valie:	Value to write in register.
 * @return:			1 in case of success, 0 in case of failure.
 */
static unsigned char __attribute__((optimize("-O0"))) MAX11136_write_register(unsigned char addr, unsigned short value) {
	unsigned short spi_command = 0;
	// Build SPI command.
	if (addr == MAX11136_REG_ADC_MODE_CONTROL) {
		// Data is 15-bits length.
		spi_command |= value & 0x00007FFF;
	}
	else {
		// Data is 11-bits length.
		spi_command |= (addr & 0x0000001F) << 11;
		spi_command |= (value & 0x000007FF);
	}
	// Send command.
	GPIO_write(&GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
#ifdef HW1_0
	unsigned char spi_access = SPI1_write_short(spi_command);
#endif
#ifdef HW2_0
	unsigned char spi_access = SPI2_write_short(spi_command);
#endif
	GPIO_write(&GPIO_MAX11136_CS, 1); // Set CS pin.
	// Return SPI access result.
	return spi_access;
}

/* PERFORM ADC CONVERSION ON ALL CHANNELS.
 * @param:	None.
 * @return:	None.
 */
static void __attribute__((optimize("-O0"))) MAX11136_convert_all_channels(void) {
#ifdef HW1_0
	// Configure SPI.
	SPI1_set_clock_polarity(1);
#endif
	// Configure ADC.
	// Single-ended unipolar (allready done at POR).
	// Enable averaging: AVGON='1' and NAVG='00' (4 conversions).
	unsigned char spi_access = MAX11136_write_register(MAX11136_REG_ADC_CONFIG, 0x0200);
	if (spi_access == 0) return;
	// Scan mode = standard internal: SCAN='0011'.
	// Perform conversion from AIN0 to AIN7: CHSEL='0111'.
	// Reset the FIFO: RESET='01'.
	// Auto shutdown: PM='01'.
	// Start conversion: SWCNV='1'.
	spi_access = MAX11136_write_register(MAX11136_REG_ADC_MODE_CONTROL, 0x1BAA);
	if (spi_access == 0) return;
	// Wait for conversions to complete.
	unsigned int loop_count = 0;
	// Wait for EOC to be pulled low.
	while (GPIO_read(&GPIO_MAX11136_EOC) != 0) {
		loop_count++;
		if (loop_count > MAX11136_TIMEOUT_COUNT) return;
	}
	// Read results in FIFO.
	unsigned short max11136_dout = 0;
	unsigned char channel = 0;
	unsigned char channel_idx = 0;
	// Wait for all channels to be read or timeout (TBD).
	for (channel_idx=0 ; channel_idx<MAX11136_NUMBER_OF_CHANNELS ; channel_idx++) {
		// Get data from SPI.
		GPIO_write(&GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
#ifdef HW1_0
		spi_access = SPI1_read_short(0x0000, &max11136_dout);
#endif
#ifdef HW2_0
		spi_access = SPI2_read_short(0x0000, &max11136_dout);
#endif
		GPIO_write(&GPIO_MAX11136_CS, 1); // Set CS pin.
		if (spi_access == 0) return;
		// Parse result = 'CH4 CH2 CH1 CH0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0'.
		channel = (max11136_dout & 0xF000) >> 12;
		if (channel < MAX11136_NUMBER_OF_CHANNELS) {
			// Fill data.
			max11136_ctx.result_12bits[channel] = max11136_dout & 0x0FFF;
			// Update status.
			max11136_ctx.status |= (0b1 << channel);
		}
	}
}

/*** MAX11136 functions ***/

/* INIT MAX11136 ADC/
 * @param:	None.
 * @return:	None.
 */
void MAX11136_init(void) {
	// Init context.
	unsigned char idx = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) max11136_ctx.result_12bits[idx] = 0;
	max11136_ctx.status = 0;
	// Configure EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* DISABLE MAX11136 GPIO.
 * @param:	None.
 * @return:	None.
 */
void MAX11136_disable_gpio(void) {
	// Disable EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* PERFORM ADC CONVERSION ON ALL CHANNELS.
 * @param:	None.
 * @return:	None.
 */
void MAX11136_perform_measurements(void) {
	// Reset results.
	unsigned char conversion_count = 0;
	for (conversion_count=0 ; conversion_count<MAX11136_NUMBER_OF_CHANNELS ; conversion_count++) max11136_ctx.result_12bits[conversion_count] = 0;
	max11136_ctx.status = 0;
	// Perform conversion until all channels are successfully retrieved or maximum number of loops is reached.
	conversion_count = 0;
	while ((max11136_ctx.status != 0xFF) && (conversion_count < MAX11136_CONVERSION_LOOPS)) {
		MAX11136_convert_all_channels();
		conversion_count++;
	}
}

/* GET CHANNEL RESULT.
 * @param channel:					ADC channel to get.
 * @param channel_voltage_12bits:	Pointer to int that will contain the channel result on 12 bits.
 * @return:							None.
 */
void MAX11136_get_data(unsigned char channel, unsigned int* channel_result_12bits) {
	// Check parameter.
	if (channel < MAX11136_NUMBER_OF_CHANNELS) {
		(*channel_result_12bits) = max11136_ctx.result_12bits[channel];
	}
}
