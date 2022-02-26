/*
 * i2c.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef I2C_H
#define I2C_H

/*** I2C functions ***/

void I2C1_init(void);
void I2C1_disable(void);
void I2C1_power_on(void);
void I2C1_power_off(void);
unsigned char I2C1_write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length, unsigned char stop_flag);
unsigned char I2C1_read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length);

#endif /* I2C_H */
