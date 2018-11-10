/*
 * sht3x.h
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludovic
 */

#ifndef SHT3X_H_
#define SHT3X_H_

/*** SHT3x functions ***/

void SHT3X_ReadTemperature(unsigned char* temperature_degrees);
void SHT3X_ReadHumidity(unsigned char* humidity_percent);

#endif /* COMPONENTS_SHT3X_H_ */
