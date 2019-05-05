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
	GPIO_BaseAddress* port_address; // GPIOA to GPIOC.
	unsigned port_index; // 0 for GPIOA, 1 for GPIOB, etc.
	unsigned char num; // 0 to 15.
	unsigned char af_num; // Alternate function number if used.
} GPIO;

typedef enum {
	Input,
	Output,
	AlternateFunction,
	Analog
} GPIO_Mode;

typedef enum {
	PushPull,
	OpenDrain
} GPIO_OutputType;

typedef enum {
	LowSpeed,
	MediumSpeed,
	HighSpeed,
	VeryHighSpeed
} GPIO_OutputSpeed;

typedef enum {
	NoPullUpNoPullDown,
	PullUp,
	PullDown
} GPIO_PullResistor;

/*** GPIO functions ***/

void GPIO_Init(void);
void GPIO_Configure(const GPIO* gpio, GPIO_Mode mode, GPIO_OutputType output_type, GPIO_OutputSpeed output_speed, GPIO_PullResistor pull_resistor);
void GPIO_Write(const GPIO* gpio, unsigned char state);
unsigned char GPIO_Read(const GPIO* gpio);
void GPIO_Toggle(const GPIO* gpio);

#endif /* GPIO_H */
