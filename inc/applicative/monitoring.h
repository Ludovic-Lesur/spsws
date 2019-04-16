/*
 * monitoring.h
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#ifndef MONITORING_H
#define MONITORING_H

/*** MONITORING macros ***/

#define MONITORING_SIGFOX_DATA_LENGTH	10

/*** MONITORING structures ***/

typedef struct {
	signed char monitoring_data_mcu_temperature_degrees;
	signed char monitoring_data_pcb_temperature_degrees;
	unsigned char monitoring_data_pcb_humidity_percent;
	unsigned int monitoring_data_solar_cell_voltage_mv;
	unsigned int monitoring_data_supercap_voltage_mv;
	unsigned int monitoring_data_mcu_voltage_mv;
	unsigned char monitoring_data_status_byte;
} MONITORING_Data;

/*** MONITORING functions ***/

void MONITORING_BuildSigfoxData(MONITORING_Data* monitoring_data, unsigned char* monitoring_sigfox_data);

#endif /* MONITORING_H */
