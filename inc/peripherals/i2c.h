/*
 * i2c.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_I2C_H
#define PERIPHERALS_I2C_H

/*** I2C macros ***/

#define I2C_MCU_ADDRESS						0x00
#define I2C_TEMPERATURE_SENSOR_ADDRESS		0b01001000

/*** I2C functions ***/

void I2C_Init(void);
void I2C_SendBytes(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length);
void I2C_GetBytes(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length);

#endif /* PERIPHERALS_I2C_H */
