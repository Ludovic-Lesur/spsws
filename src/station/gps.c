/*
 * gps.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#include "gpio_reg.h"
#include "gps.h"
#include "lptim.h"
#include "lpuart.h"
#include "rcc_reg.h"

/*** GPS local macros ***/

#define LOCAL_UTC_OFFSET					2 // TBD: configurable via downlink.

#define NMEA_CR								'\r'
#define NMEA_LF								'\n'
#define NMEA_SEP							','
#define NMEA_DOT							'.'
#define NMEA_NUMBER_OF_ID_TO_DISABLE		17
#define NMEA_RX_BUFFER_SIZE					128

#define NMEA_GGA_ADDRESS_FIELD_LENGTH		6
#define NMEA_GGA_TIME_FIELD_LENGTH			9
#define NMEA_GGA_LAT_FIELD_LENGTH			10
#define NMEA_GGA_NS_FIELD_LENGTH			1
#define NMEA_GGA_NORTH						'N'
#define NMEA_GGA_SOUTH						'S'
#define NMEA_GGA_LONG_FIELD_LENGTH			11
#define NMEA_GGA_EO_FIELD_LENGTH			1
#define NMEA_GGA_EAST						'E'
#define NMEA_GGA_WEST						'O'

#define UBX_MSG_OVERHEAD_LENGTH				8
#define UBX_CHECKSUM_OVERHEAD_LENGTH		4
#define UBX_CHECKSUM_OFFSET					2

#define UBX_CFG_MSG_PAYLOAD_LENGTH			8

/*** GPS local structures ***/

typedef struct {
	unsigned char nmea_rx_buf[NMEA_RX_BUFFER_SIZE]; 	// NMEA input messages buffer.
	unsigned int nmea_rx_buf_idx;						// Current index in RX buffer.
	unsigned int nmea_frame_length;
	unsigned char nmea_lf_flag;					// Set to '1' as soon as a complete NMEA message is received.
} GPS_Context;

typedef struct {
	/* Time */
	unsigned char time_hours;
	unsigned char time_minutes;
	unsigned char time_seconds;
	/* Latitude */
	unsigned char lat_degrees;
	unsigned char lat_minutes;
	unsigned int lat_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char lat_north; // 0='S', 1='N'.
	/* Longitude */
	unsigned char long_degrees;
	unsigned char long_minutes;
	unsigned int long_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char long_east; // 0='O', 1='E'.
	/* Altitude */
	unsigned int altitude;
} NMEA_Data;

/*** GPS local global variables ***/

static GPS_Context gps_ctx;
static NMEA_Data nmea_data;

/*** GPS local functions ***/

/* COMPUTE AND APPEND CHECKSUM TO AN UBX MESSAGE.
 * @param ubx_command:		Complete UBX message for which checksum must be computed.
 * @param payload_length:	Length of the payload (in bytes) for this message.
 * @return:					None.
 */
void GPS_UbxChecksum(unsigned char* ubx_command, unsigned char payload_length) {
	// See algorithme on p.136 of NEO-M8 programming manual.
	unsigned char ck_a = 0;
	unsigned char ck_b = 0;
	unsigned char checksum_idx = 0;
	for (checksum_idx=UBX_CHECKSUM_OFFSET ; checksum_idx<(UBX_CHECKSUM_OFFSET+UBX_CHECKSUM_OVERHEAD_LENGTH+payload_length) ; checksum_idx++) {
		ck_a = ck_a + ubx_command[checksum_idx];
		ck_b = ck_b + ck_a;
	}
	// Fill two last bytes of the UBX message with CK_A and CK_B.
	ubx_command[checksum_idx] = ck_a;
	ubx_command[checksum_idx+1] = ck_b;
}

/* CONVERTS THE ASCII CODE OF A DECIMAL CHARACTER TO THE CORRESPONDING VALUE.
 * @param c:			Hexadecimal character to convert.
 * @return hexa_value:	Result of conversion.
 */
unsigned char AsciiToDecimal(unsigned char c) {
	unsigned char value = 0;
	if ((c >= '0') && (c <= '9')) {
		value = c - '0';
	}
	return value;
}

/* COMPUTE A POWER A 10.
 * @param power:	The desired power.
 * @return result:	Result of computation.
 */
unsigned int Pow10(unsigned char n) {
	unsigned int result = 0;
	unsigned int pow10_buf[9] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
	if (n <= 9) {
		result = pow10_buf[n];
	}
	return result;
}

/*** GPS functions ***/

/* CONFIGURE GPS MODULE (U-BLOX NEO-M8N).
 * @param:	None.
 * @return:	NOne.
 */
void GPS_Init(void) {

	/* Init context */
	unsigned int idx = 0;
	for (idx=0 ; idx<NMEA_RX_BUFFER_SIZE ; idx++) gps_ctx.nmea_rx_buf[idx] = 0;
	gps_ctx.nmea_rx_buf_idx = 0;
	gps_ctx.nmea_frame_length = 0;
	gps_ctx.nmea_lf_flag = 0;

	/* Init NMEA data */
	nmea_data.time_hours = 0;
	nmea_data.time_minutes = 0;
	nmea_data.time_seconds = 0;
	nmea_data.lat_degrees = 0;
	nmea_data.lat_minutes = 0;
	nmea_data.lat_seconds = 0;
	nmea_data.lat_north = 0;
	nmea_data.long_degrees = 0;
	nmea_data.long_minutes = 0;
	nmea_data.long_seconds = 0;
	nmea_data.long_east = 0;
	nmea_data.altitude = 0;

	/* Switch GPS module on */
	RCC -> IOPENR |= (0b1 << 1); // Enable GPIOB clock.
	GPIOB -> MODER &= ~(0b11 << 10); // Reset bits 10-11.
	GPIOB -> MODER |= (0b01 << 10); // Configure PB5 as output.
	GPIOB -> ODR |= (0b1 << 5); // Switch on GPS.

	/* Wait 2s to skip boot messages */
	LPTIM_WaitMs(2000);

	/* Init LPUART */
	LPUART_Init();
	LPUART_EnableTx();

	/* Send UBX commands to get GGA messages only */
	// See p.110 for NMEA messages ID.
	unsigned char nmea_id_to_disable[NMEA_NUMBER_OF_ID_TO_DISABLE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0F, 0x40, 0x41, 0x42, 0x43, 0x44};
	unsigned char nmea_id_idx = 0;
	// See p.174 for UBX message format.
	unsigned char ubx_cfg_msg[UBX_MSG_OVERHEAD_LENGTH+UBX_CFG_MSG_PAYLOAD_LENGTH] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char ubx_cfg_msg_idx = 0;
	// Send commands.
	for (nmea_id_idx=0 ; nmea_id_idx<NMEA_NUMBER_OF_ID_TO_DISABLE ; nmea_id_idx++) {
		ubx_cfg_msg[7] = nmea_id_to_disable[nmea_id_idx]; // 7th byte is the ID of the message to disable.
		GPS_UbxChecksum(ubx_cfg_msg, UBX_CFG_MSG_PAYLOAD_LENGTH); // Compute checksum (CK_A and CK_B).
		for (ubx_cfg_msg_idx=0 ; ubx_cfg_msg_idx<(UBX_MSG_OVERHEAD_LENGTH+UBX_CFG_MSG_PAYLOAD_LENGTH) ; ubx_cfg_msg_idx++) {
			LPUART_SendByte(ubx_cfg_msg[ubx_cfg_msg_idx]); // Send
		}
	}

	/* Wait 1s to skip ACK messages */
	LPTIM_WaitMs(1000);

	/* Start reception and wait for GPS to fix*/
	LPUART_EnableRx();
}

/* FILL NMEA RX BUFFER (CALLED BY LPUART RXNE INTERRUPT).
 * @param new_byte:		New byte to store.
 * @return:				None.
 */
void GPS_FillNmeaRxBuffer(unsigned char new_byte) {

	/* Fill NMEA RX buffer with incomming byte */
	gps_ctx.nmea_rx_buf[gps_ctx.nmea_rx_buf_idx] = new_byte;

	/* Detect LF character (end of frame in NMEA protocol) */
	if (new_byte == NMEA_LF) {
		gps_ctx.nmea_lf_flag = 1; // Set LF flag to start decoding.
		gps_ctx.nmea_frame_length = gps_ctx.nmea_rx_buf_idx; // Save current frame length.
		gps_ctx.nmea_rx_buf_idx = 0; // Reset RX index.
	}
	else {
		gps_ctx.nmea_rx_buf_idx++; // Increment RX index.
		if (gps_ctx.nmea_rx_buf_idx == NMEA_RX_BUFFER_SIZE) {
			gps_ctx.nmea_rx_buf_idx = 0; // Manage roll-over.
		}
	}
}

/* MAIN ROUTINE OF GPS MODULE.
 * @param:			None.
 * @return success:	Boolean indicating the state of GPS messages decoding.
 * 					0 = fix not complete.
 * 					1 = all NMEA fields retrieved.
 */
unsigned char GPS_Processing(void) {
	unsigned char error_found = 0;
	unsigned char success = 0;
	/* Start decoding if complete NMEA message is received */
	if (gps_ctx.nmea_lf_flag == 1) {

		/* Extract NMEA data (see GGA message format on p.114 of NEO-M8 programming manual) */
		unsigned char idx = 0;
		unsigned char sep_idx = 0;
		unsigned char sep_range = 0;
		unsigned char alt_field_length = 0;
		unsigned char alt_number_of_digits = 0;
		for (idx=0 ; idx<gps_ctx.nmea_frame_length ; idx++) {
			if (gps_ctx.nmea_rx_buf[idx] == NMEA_SEP) {
				sep_range++;
				unsigned int k = 0; // Generic index used in local for loops.
				switch (sep_range) {

				/* Field 1 = address = <ID><message*/
				case 1:

					if (idx == NMEA_GGA_ADDRESS_FIELD_LENGTH) {
						/* Check if message = 'GGA' */
						if ((gps_ctx.nmea_rx_buf[3] != 'G') || (gps_ctx.nmea_rx_buf[4] != 'G') || (gps_ctx.nmea_rx_buf[5] != 'A')) {
							error_found = 1;
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Field 2 = time = <hhmmss.ss> */
				case 2:
					if ((idx-sep_idx) == (NMEA_GGA_TIME_FIELD_LENGTH+1)) {
						nmea_data.time_hours = LOCAL_UTC_OFFSET + AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+1])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+2]);
						nmea_data.time_minutes = AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+3])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+4]);
						nmea_data.time_seconds = AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+5])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+6]);
					}
					else {
						error_found = 1;
					}
					break;

					/* Field 3 = latitude = <ddmm.mmmmm> */
				case 3:
					if ((idx-sep_idx) == (NMEA_GGA_LAT_FIELD_LENGTH+1)) {
						nmea_data.lat_degrees = AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+1])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+2]);
						nmea_data.lat_minutes = AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+3])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+4]);
						for (k=0 ; k<5 ; k++) {
							nmea_data.lat_seconds = nmea_data.lat_seconds + Pow10(4-k)*AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+6+k]);
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Field 4 = <N> or <S> */
				case 4:
					if ((idx-sep_idx) == (NMEA_GGA_NS_FIELD_LENGTH+1)) {
						switch (gps_ctx.nmea_rx_buf[sep_idx+1]) {
						case NMEA_GGA_NORTH:
							nmea_data.lat_north = 1;
							break;
						case NMEA_GGA_SOUTH:
							nmea_data.lat_north = 0;
							break;
						default:
							nmea_data.lat_north = 0;
							error_found = 1;
							break;
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Field 5 = longitude = <dddmm.mmmmm */
				case 5:
					if ((idx-sep_idx) == (NMEA_GGA_LONG_FIELD_LENGTH+1)) {
						for (k=0 ; k<3 ; k++) {
							nmea_data.long_degrees = nmea_data.long_degrees + Pow10(2-k)*AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+1+k]);
						}
						nmea_data.long_minutes = AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+4])*10+AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+5]);
						for (k=0 ; k<5 ; k++) {
							nmea_data.long_seconds = nmea_data.long_seconds + Pow10(4-k)*AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+7+k]);
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Field 6 = <E> or <O> */
				case 6:
					if ((idx-sep_idx) == (NMEA_GGA_EO_FIELD_LENGTH+1)) {
						switch (gps_ctx.nmea_rx_buf[sep_idx+1]) {
						case NMEA_GGA_EAST:
							nmea_data.long_east = 1;
							break;
						case NMEA_GGA_WEST:
							nmea_data.long_east = 0;
							break;
						default:
							nmea_data.long_east = 0;
							error_found = 1;
							break;
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Field 10 = altitude */
				case 10:
					alt_field_length = (idx-sep_idx)-1;
					if (alt_field_length >= 1) {

						/* Get number of digits of integer part (search dot) */
						for (alt_number_of_digits=0 ; alt_number_of_digits<alt_field_length ; alt_number_of_digits++) {
							if (gps_ctx.nmea_rx_buf[sep_idx+1+alt_number_of_digits] == NMEA_DOT) {
								break; // Dot found, stop counting integer part length.
							}
						}

						/* Compute integer part */
						if (alt_number_of_digits > 0) {
							for (k=0 ; k<alt_number_of_digits ; k++) {
								nmea_data.altitude = nmea_data.altitude + Pow10(alt_number_of_digits-1-k)*AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+1+k]);
							}

							/* Rounding operation if fractionnal part exists (not required for success) */
							if ((idx-(sep_idx+alt_number_of_digits)-1) >= 2) {
								if (AsciiToDecimal(gps_ctx.nmea_rx_buf[sep_idx+alt_number_of_digits+2]) >= 5) {
									nmea_data.altitude++; // Add '1' to altitude.
								}
							}
							success = 1; // Last field retrieved, decoding process successed.
						}
						else {
							error_found = 1;
						}
					}
					else {
						error_found = 1;
					}
					break;

				/* Unused fields */
				default:
					break;
				}
				sep_idx = idx; // Update separator index.
			}

			/* Check syntax error flag */
			if (error_found == 1) {
				break; // Exit decoding loop as soon as an error occured.
			}
		}
	}
	return success;
}
