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

/*** GPIO structures ***/

typedef struct {
	GPIO_base_address_t* port_address; // GPIOA to GPIOC.
	uint8_t port_index; // 0 for GPIOA, 1 for GPIOB, etc.
	uint8_t pin_index; // 0 to 15.
	uint8_t alternate_function_index; // Alternate function number if used.
} GPIO_pin_t;

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE_FUNCTION,
	GPIO_MODE_ANALOG,
	GPIO_MODE_LAST
} GPIO_mode_t;

typedef enum {
	GPIO_TYPE_PUSH_PULL,
	GPIO_TYPE_OPEN_DRAIN,
	GPIO_TYPE_LAST
} GPIO_output_type_t;

typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH,
	GPIO_SPEED_LAST
} GPIO_output_speed_t;

typedef enum {
	GPIO_PULL_NONE,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
	GPIO_PULL_LAST
} GPIO_pull_resistor_t;

/*** GPIO functions ***/

void GPIO_init(void);
void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor);
void GPIO_write(const GPIO_pin_t* gpio, uint8_t state);
uint8_t GPIO_read(const GPIO_pin_t* gpio);
void GPIO_toggle(const GPIO_pin_t* gpio);

#endif /* __GPIO_H__ */
