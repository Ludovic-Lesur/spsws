/*
 * gpio.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#include "gpio_reg.h"
#include "types.h"

/*** GPIO macros ***/

#define GPIO_PINS_PER_PORT	16

/*** GPIO structures ***/

/*!******************************************************************
 * \enum GPIO_pin_t
 * \brief GPIO pin descriptor.
 *******************************************************************/
typedef struct {
	GPIO_registers_t* port;
	uint8_t port_index;
	uint8_t pin;
	uint8_t alternate_function;
} GPIO_pin_t;

/*!******************************************************************
 * \enum GPIO_mode_t
 * \brief GPIO modes list.
 *******************************************************************/
typedef enum {
	GPIO_MODE_INPUT = 0,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE_FUNCTION,
	GPIO_MODE_ANALOG,
	GPIO_MODE_LAST
} GPIO_mode_t;

/*!******************************************************************
 * \enum GPIO_output_type_t
 * \brief GPIO output types.
 *******************************************************************/
typedef enum {
	GPIO_TYPE_PUSH_PULL = 0,
	GPIO_TYPE_OPEN_DRAIN,
	GPIO_TYPE_LAST
} GPIO_output_type_t;

/*!******************************************************************
 * \enum GPIO_output_speed_t
 * \brief GPIO output speeds.
 *******************************************************************/
typedef enum {
	GPIO_SPEED_LOW = 0,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH,
	GPIO_SPEED_LAST
} GPIO_output_speed_t;

/*!******************************************************************
 * \enum GPIO_pull_resistor_t
 * \brief GPIO pull resistor configurations.
 *******************************************************************/
typedef enum {
	GPIO_PULL_NONE = 0,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
	GPIO_PULL_LAST
} GPIO_pull_resistor_t;

/*** GPIO functions ***/

/*!******************************************************************
 * \fn void GPIO_init(void)
 * \brief Init GPIO driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void GPIO_init(void);

/*!******************************************************************
 * \fn void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor)
 * \brief Configure a GPIO.
 * \param[in]  	gpio: GPIO to configure.
 * \param[in]	mode: GPIO mode.
 * \param[in]	output_type: GPIO output type.
 * \param[in]	output_speed: GPIO output speed.
 * \param[in]	pull_resistor: GPIO pull resistor configuration.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor);

/*!******************************************************************
 * \fn void GPIO_write(const GPIO_pin_t* gpio, uint8_t state)
 * \brief Set GPIO output state.
 * \param[in]  	gpio: GPIO to write.
 * \param[in]	state: Output state to write.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void GPIO_write(const GPIO_pin_t* gpio, uint8_t state);

/*!******************************************************************
 * \fn uint8_t GPIO_read(const GPIO_pin_t* gpio)
 * \brief Read GPIO input or output state.
 * \param[in]  	gpio: GPIO to read.
 * \param[out] 	none
 * \retval		0 for low logic voltage, 1 for high logic voltage.
 *******************************************************************/
uint8_t GPIO_read(const GPIO_pin_t* gpio);

/*!******************************************************************
 * \fn void GPIO_toggle(const GPIO_pin_t* gpio)
 * \brief Toggle GPIO output state.
 * \param[in]  	gpio: GPIO to toggle.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void GPIO_toggle(const GPIO_pin_t* gpio);

#endif /* __GPIO_H__ */
