/*
 * sht3x.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludovic
 */

#include "sht3x.h"

#include "i2c.h"

/*** SHT3x local macros ***/

#define SHT3X_I2C_ADDRESS	0x44

/*** SHT3x functions ***/

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (°C).
 * @return:						None.
 */
void SHT3X_ReadTemperature(unsigned char* temperature_degrees) {

	/* Trigger high repeatability measurement with clock streching disabled */
	unsigned char measurement_command[2] = {0x24, 0x00};
	I2C_Write(SHT3X_I2C_ADDRESS, measurement_command, 2);
}

/* READ HUMIDITY FROM SHT3X SENSOR.
 * @param humidity_percent:	Pointer to byte that will contain humidity result (%).
 * @return:					None.
 */
void SHT3X_ReadHumidity(unsigned char* humidity_percent) {

}
