/*
 * i2c.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef I2C_H
#define I2C_H

/*** I2C functions ***/

void I2C1_Init(void);
void I2C1_Disable(void);
void I2C1_PowerOn(void);
void I2C1_PowerOff(void);
unsigned char I2C1_Write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length);
unsigned char I2C1_Read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length);

#endif /* I2C_H */
