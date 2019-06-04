/*
 * gpio.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef GPIO_H
#define GPIO_H

#include "gpio_reg.h"

/*** GPIO structures ***/

// GPIO structure.
typedef struct {
	GPIO_BaseAddress* gpio_port_address; // GPIOA to GPIOC.
	unsigned char gpio_port_index; // 0 for GPIOA, 1 for GPIOB, etc.
	unsigned char gpio_num; // 0 to 15.
	unsigned char gpio_af_num; // Alternate function number if used.
} GPIO;

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE_FUNCTION,
	GPIO_MODE_ANALOG
} GPIO_Mode;

typedef enum {
	GPIO_TYPE_PUSH_PULL,
	GPIO_TYPE_OPEN_DRAIN
} GPIO_OutputType;

typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH,
} GPIO_OutputSpeed;

typedef enum {
	GPIO_PULL_NONE,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN
} GPIO_PullResistor;

/*** GPIO functions ***/

void GPIO_Init(void);
void GPIO_Configure(const GPIO* gpio, GPIO_Mode mode, GPIO_OutputType output_type, GPIO_OutputSpeed output_speed, GPIO_PullResistor pull_resistor);
void GPIO_Write(const GPIO* gpio, unsigned char state);
unsigned char GPIO_Read(const GPIO* gpio);
void GPIO_Toggle(const GPIO* gpio);

#endif /* GPIO_H */
