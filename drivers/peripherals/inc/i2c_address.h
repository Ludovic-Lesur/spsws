/*
 * i2c_address.h
 *
 *  Created on: 27 aug. 2024
 *      Author: Ludo
 */

#ifndef __I2C_ADDRESS_H__
#define __I2C_ADDRESS_H__

/*!******************************************************************
 * \enum I2C_address_mapping_t
 * \brief I2C slaves address mapping.
 *******************************************************************/
typedef enum {
    I2C_ADDRESS_SHT30_INTERNAL = 0x44,
    I2C_ADDRESS_SHT30_EXTERNAL = 0x45,
    I2C_ADDRESS_DPS310 = 0x77,
    I2C_ADDRESS_SI1133 = 0x52
} I2C_address_mapping_t;

#endif /* __I2C_ADDRESS_H__ */
