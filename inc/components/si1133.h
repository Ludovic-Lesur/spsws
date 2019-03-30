/*
 * si1133.h
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#ifndef SI1133_H
#define SI1133_H

/*** SI1133 macros ***/

#define SI1133_EXTERNAL_I2C_ADDRESS		0x52

/*** SI1133 functions ***/

void SI1133_Init(void);
void SI1133_PerformMeasurements(unsigned char si1133_i2c_address);
void SI1133_GetUvIndex(unsigned char* uv_index);

#endif /* SI1133_H */
