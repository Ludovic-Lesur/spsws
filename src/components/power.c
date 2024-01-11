/*
 * power.c
 *
 *  Created on: 12 dec. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lptim.h"
#include "mapping.h"
#include "max11136.h"
#include "neom8n.h"
#include "sky13317.h"
#include "spi.h"
#include "sx1232.h"
#include "types.h"

/*** POWER local global variables ***/

static uint8_t power_domain_state[POWER_DOMAIN_LAST];

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
	// Local variables.
	POWER_domain_t domain = 0;
	// Init power control pins.
	GPIO_configure(&GPIO_TCXO16_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef HW2_0
	GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	GPIO_configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_TCXO32_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable all domains by default.
	for (domain=0 ; domain<POWER_DOMAIN_LAST ; domain++) {
		POWER_disable(domain);
	}
}

/*******************************************************************/
POWER_status_t POWER_enable(POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t delay_ms = 0;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG_INTERNAL:
		// Init internal ADC.
		adc1_status = ADC1_init();
		ADC1_exit_error(POWER_ERROR_BASE_ADC1);
		delay_ms = POWER_ON_DELAY_MS_ANALOG_INTERNAL;
		break;
	case POWER_DOMAIN_ANALOG_EXTERNAL:
		// Turn analog front-end on and init external ADC.
#ifdef HW1_0
		GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
		GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
		SX1232_init();
		I2C1_init();
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
#endif
		MAX11136_init();
#ifdef HW1_0
		SPI1_set_clock_configuration(1, 1);
#endif
		delay_ms = POWER_ON_DELAY_MS_ANALOG_EXTERNAL;
		break;
	case POWER_DOMAIN_SENSORS:
		// Turn digital sensors on and init common I2C interface.
		GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
#ifdef HW1_0
		GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
		MAX11136_init();
		SX1232_init();
#endif
		I2C1_init();
		delay_ms = POWER_ON_DELAY_MS_SENSORS;
		break;
	case POWER_DOMAIN_GPS:
		// Turn GPS on and init NEOM8N driver.
		GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
		NEOM8N_init();
		delay_ms = POWER_ON_DELAY_MS_GPS;
		break;
	case POWER_DOMAIN_RADIO_TCXO:
		// Turn radio TCXO on.
		GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 1);
		delay_ms = POWER_ON_DELAY_MS_RADIO_TCXO;
		break;
	case POWER_DOMAIN_RADIO:
#ifdef HW1_0
		GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
		MAX11136_init();
		I2C1_init();
#endif
		// Turn radio on and init SX1232 driver.
		GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
		SX1232_init();
#ifdef HW1_0
		SPI1_set_clock_configuration(0, 0);
#endif
		SKY13317_init();
		delay_ms = POWER_ON_DELAY_MS_RADIO;
		break;
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 1;
	// Power on delay.
	if (delay_ms != 0) {
		lptim1_status = LPTIM1_delay_milliseconds(delay_ms, delay_mode);
		LPTIM1_exit_error(POWER_ERROR_BASE_LPTIM);
	}
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_disable(POWER_domain_t domain) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG_INTERNAL:
		// Release internal ADC.
		adc1_status = ADC1_de_init();
		ADC1_exit_error(POWER_ERROR_BASE_ADC1);
		break;
	case POWER_DOMAIN_ANALOG_EXTERNAL:
		// Turn analog front-end off and release external ADC.
#ifdef HW1_0
		if ((power_domain_state[POWER_DOMAIN_RADIO] == 0) && (power_domain_state[POWER_DOMAIN_SENSORS] == 0)) {
			GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
			GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
			SX1232_de_init();
			I2C1_de_init();
		}
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
#endif
		MAX11136_de_init();
		break;
	case POWER_DOMAIN_SENSORS:
#ifdef HW1_0
		if ((power_domain_state[POWER_DOMAIN_RADIO] == 0) && (power_domain_state[POWER_DOMAIN_ANALOG_EXTERNAL] == 0)) {
			GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
			GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
			MAX11136_de_init();
			SX1232_de_init();
		}
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
#endif
		// Turn digital sensors off and release I2C interface.
		I2C1_de_init();
		break;
	case POWER_DOMAIN_GPS:
		// Turn GPS off and release NEOM8N driver.
		NEOM8N_de_init();
		GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
		break;
	case POWER_DOMAIN_RADIO_TCXO:
		// Turn radio TCXO off.
		GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 0);
		break;
	case POWER_DOMAIN_RADIO:
#ifdef HW1_0
		if ((power_domain_state[POWER_DOMAIN_SENSORS] == 0) && (power_domain_state[POWER_DOMAIN_ANALOG_EXTERNAL] == 0)) {
			GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
			GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
			MAX11136_de_init();
			I2C1_de_init();
		}
#endif
#ifdef HW2_0
		GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
#endif
		// Turn radio off and release SX1232 driver.
		SX1232_de_init();
		SKY13317_de_init();
		break;
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 0;
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_get_state(POWER_domain_t domain, uint8_t* state) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	// Check parameters.
	if (domain >= POWER_DOMAIN_LAST) {
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	if (state == NULL) {
		status = POWER_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*state) = power_domain_state[domain];
errors:
	return status;
}
