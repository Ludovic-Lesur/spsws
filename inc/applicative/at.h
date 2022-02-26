/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "mode.h"

#ifdef ATM

/*** AT functions ***/

void AT_init(void);
void AT_task(void);
void AT_fill_rx_buffer(unsigned char rx_byte);
void AT_print_test_result(unsigned char status, int rssi);
void AT_print_rain(unsigned char rain_edge_count);
void AT_print_wind_speed(unsigned int speed_mh);
void AT_print_wind_direction(unsigned int direction_degrees, int direction_x, int direction_y);

#endif

#endif /* AT_H */
