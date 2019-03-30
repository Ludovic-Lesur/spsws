/*
 * monitoring.c
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#include "monitoring.h"

/*** MONITORING functions ***/

/* BUILD SIGFOX UPLINK MONITORING DATA FRAME.
 * @param monitoring_data:			Raw data structure retrieved from sensors.
 * @param monitoring_sigfox_data:	Byte array that will contain Sigfox data.
 * @return:							None.
 */
void MONITORING_BuildSigfoxData(MONITORING_Data* monitoring_data, unsigned char* monitoring_sigfox_data) {
	// MCU temperature (°C).
	monitoring_sigfox_data[0] = monitoring_data -> monitoring_data_mcu_temperarure_degrees;
	// PCB temperature (°C).
	monitoring_sigfox_data[1] = monitoring_data -> monitoring_data_pcb_temperarure_degrees;
	// PCB humidity (%).
	monitoring_sigfox_data[2] = monitoring_data -> monitoring_data_pcb_humidity_percent;
	// Solar cell voltage (mV).
	monitoring_sigfox_data[3] = ((monitoring_data -> monitoring_data_solar_cell_voltage_mv) & 0xFF00) >> 8;
	monitoring_sigfox_data[4] = ((monitoring_data -> monitoring_data_solar_cell_voltage_mv) & 0x00FF) >> 0;
	// Supercap voltage (mV).
	monitoring_sigfox_data[5] = ((monitoring_data -> monitoring_data_supercap_voltage_mv) & 0xFF00) >> 8;
	monitoring_sigfox_data[6] = ((monitoring_data -> monitoring_data_supercap_voltage_mv) & 0x00FF) >> 0;
	// MCU voltage (mV).
	monitoring_sigfox_data[7] = ((monitoring_data -> monitoring_data_mcu_voltage_mv) & 0xFF00) >> 8;
	monitoring_sigfox_data[8] = ((monitoring_data -> monitoring_data_mcu_voltage_mv) & 0x00FF) >> 0;
	// Status byte.
	monitoring_sigfox_data[9] = monitoring_data -> monitoring_data_status_byte;
}
