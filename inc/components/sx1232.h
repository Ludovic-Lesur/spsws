/*
 * sx1232.h
 *
 *  Created on: 20 june 2018
 *      Author: Ludovic
 */

#ifndef COMPONENTS_SX1232_H_
#define COMPONENTS_SX1232_H_

/*** SX1232 functions ***/

void SX1232_WriteRegister(unsigned char addr, unsigned char value);
void SX1232_ReadRegister(unsigned char addr, unsigned char* value);
void SX1232_Init(void);
void SX1232_Start(void);

#endif /* COMPONENTS_SX1232_H_ */
