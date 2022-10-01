/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef __AT_H__
#define __AT_H__

#include "mode.h"
#include "types.h"

#ifdef ATM

/*** AT functions ***/

void AT_init(void);
void AT_task(void);
void AT_fill_rx_buffer(uint8_t rx_byte);
void AT_print_test_result(uint8_t status, int32_t rssi);
void AT_print_rain(uint8_t rain_edge_count);
void AT_print_wind_speed(uint32_t speed_mh);
void AT_print_wind_direction(uint32_t direction_degrees, int32_t direction_x, int32_t direction_y);

#endif

#endif /* __AT_H__ */
