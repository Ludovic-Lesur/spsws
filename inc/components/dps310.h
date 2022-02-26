/*
 * dps310.h
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludo
 */

#ifndef DPS310_H
#define DPS310_H

/*** DPS310 macros ***/

#define DPS310_EXTERNAL_I2C_ADDRESS		0x77
#define DPS310_PRESSURE_ERROR_VALUE		0xFFFFFFFF

/*** DPS310 functions ***/

void DPS310_init(void);
void DPS310_perform_measurements(unsigned char dps310_i2c_address);
void DPS310_get_pressure(unsigned int* pressure_pa);
void DPS310_get_temperature(signed char* temperature_degrees);

#endif /* DPS310_H */
