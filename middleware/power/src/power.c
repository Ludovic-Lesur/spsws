/*
 * power.c
 *
 *  Created on: 12 dec. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "dps310.h"
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "gps.h"
#include "lptim.h"
#include "rfe.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
#include "types.h"

/*** POWER local global variables ***/

static uint32_t power_domain_state[POWER_DOMAIN_LAST] = { [0 ... (POWER_DOMAIN_LAST - 1)] = 0 };

/*** POWER local functions ***/

/*******************************************************************/
#define _POWER_stack_driver_error(driver_status, driver_success, driver_error_base, power_status) { \
    if (driver_status != driver_success) { \
        ERROR_stack_add(driver_error_base + driver_status); \
        ERROR_stack_add(ERROR_BASE_POWER + power_status); \
    } \
}

/*******************************************************************/
static void _POWER_analog_init(void) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    // Turn external ADC on.
#ifdef HW1_0
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
#endif
#ifdef HW2_0
    GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
#endif
    // Init internal and external ADC drivers.
    analog_status = ANALOG_init();
    _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
}

/*******************************************************************/
static void _POWER_analog_de_init(void) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    // Release internal and external ADC drivers.
    analog_status = ANALOG_de_init();
    _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
    // Turn external ADC off.
#ifdef HW1_0
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
#endif
#ifdef HW2_0
    GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
#endif
}

/*******************************************************************/
static void _POWER_sensors_init(void) {
    // Local variables.
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    // Turn sensors on.
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
    // Init sensors drivers.
    dps310_status = DPS310_init();
    _POWER_stack_driver_error(dps310_status, DPS310_SUCCESS, ERROR_BASE_DPS310, POWER_ERROR_DRIVER_DPS310);
    sht3x_status = SHT3X_init();
    _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30_INTERNAL, POWER_ERROR_DRIVER_SHT3X);
    si1133_status = SI1133_init();
    _POWER_stack_driver_error(si1133_status, SI1133_SUCCESS, ERROR_BASE_SI1133, POWER_ERROR_DRIVER_SI1133);
}

/*******************************************************************/
static void _POWER_sensors_de_init(void) {
    // Local variables.
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    // Release sensors drivers.
    dps310_status = DPS310_de_init();
    _POWER_stack_driver_error(dps310_status, DPS310_SUCCESS, ERROR_BASE_DPS310, POWER_ERROR_DRIVER_DPS310);
    sht3x_status = SHT3X_de_init();
    _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30_INTERNAL, POWER_ERROR_DRIVER_SHT3X);
    si1133_status = SI1133_de_init();
    _POWER_stack_driver_error(si1133_status, SI1133_SUCCESS, ERROR_BASE_SI1133, POWER_ERROR_DRIVER_SI1133);
    // Turn sensors off.
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
}

/*******************************************************************/
static void _POWER_gps_init(void) {
    // Local variables.
    GPS_status_t gps_status = GPS_SUCCESS;
    // Turn GPS on.
    GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
    // Init GPS driver.
    gps_status = GPS_init();
    _POWER_stack_driver_error(gps_status, GPS_SUCCESS, ERROR_BASE_GPS, POWER_ERROR_DRIVER_GPS);
}

/*******************************************************************/
static void _POWER_gps_de_init(void) {
    // Local variables.
    GPS_status_t gps_status = GPS_SUCCESS;
    // Release GPS driver.
    gps_status = GPS_de_init();
    _POWER_stack_driver_error(gps_status, GPS_SUCCESS, ERROR_BASE_GPS, POWER_ERROR_DRIVER_GPS);
    // Turn GPS off.
    GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
}

/*******************************************************************/
static void _POWER_radio_init(void) {
    // Local variables.
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Turn radio on.
    GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
    // Init transceiver and front-end.
    sx1232_status = SX1232_init();
    _POWER_stack_driver_error(sx1232_status, SX1232_SUCCESS, ERROR_BASE_SX1232, POWER_ERROR_DRIVER_SX1232);
    rfe_status = RFE_init();
    _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
}

/*******************************************************************/
static void _POWER_radio_de_init(void) {
    // Local variables.
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Release transceiver and front-end.
    sx1232_status = SX1232_de_init();
    _POWER_stack_driver_error(sx1232_status, SX1232_SUCCESS, ERROR_BASE_SX1232, POWER_ERROR_DRIVER_SX1232);
    rfe_status = RFE_de_init();
    _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
    // Turn radio off.
    GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
}

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
    // Local variables.
    uint8_t idx = 0;
    // Init context.
    for (idx = 0; idx < POWER_DOMAIN_LAST; idx++) {
        power_domain_state[idx] = 0;
    }
    // Init power control pins.
    GPIO_configure(&GPIO_TCXO16_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef HW2_0
    GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
    GPIO_configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_TCXO32_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*******************************************************************/
void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint32_t delay_ms = 0;
    uint8_t action_required = 0;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    action_required = ((power_domain_state[domain] == 0) ? 1 : 0);
    // Update state.
    power_domain_state[domain] |= (0b1 << requester_id);
    // Directly exit if this is not the first request.
    if (action_required == 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_MCU_TCXO:
        // Turn MCU TCXO on.
        GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_MCU_TCXO;
        break;
    case POWER_DOMAIN_ANALOG:
        // Turn analog front-end on and init attached drivers.
#ifdef HW1_0
        if (power_domain_state[POWER_DOMAIN_RADIO] == 0) {
            _POWER_radio_init();
            power_domain_state[POWER_DOMAIN_RADIO] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
        if (power_domain_state[POWER_DOMAIN_SENSORS] == 0) {
            _POWER_sensors_init();
            power_domain_state[POWER_DOMAIN_SENSORS] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
#endif
        _POWER_analog_init();
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
        break;
    case POWER_DOMAIN_SENSORS:
        // Turn digital sensors and init attached drivers.
#ifdef HW1_0
        if (power_domain_state[POWER_DOMAIN_ANALOG] == 0) {
            _POWER_analog_init();
            power_domain_state[POWER_DOMAIN_ANALOG] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
        if (power_domain_state[POWER_DOMAIN_RADIO] == 0) {
            _POWER_radio_init();
            power_domain_state[POWER_DOMAIN_RADIO] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
#endif
        _POWER_sensors_init();
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        break;
    case POWER_DOMAIN_GPS:
        // Turn GPS on and init attached drivers.
        _POWER_gps_init();
        delay_ms = POWER_ON_DELAY_MS_GPS;
        break;
    case POWER_DOMAIN_RADIO_TCXO:
        // Turn radio TCXO on.
        GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_RADIO_TCXO;
        break;
    case POWER_DOMAIN_RADIO:
#ifdef HW1_0
        // Turn radio on and init attached drivers.
        if (power_domain_state[POWER_DOMAIN_ANALOG] == 0) {
            GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
            _POWER_analog_init();
            power_domain_state[POWER_DOMAIN_ANALOG] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
        if (power_domain_state[POWER_DOMAIN_SENSORS] == 0) {
            _POWER_sensors_init();
            power_domain_state[POWER_DOMAIN_SENSORS] |= (0b1 << POWER_REQUESTER_ID_POWER);
        }
#endif
        _POWER_radio_init();
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        _POWER_stack_driver_error(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM, POWER_ERROR_DRIVER_LPTIM);
    }
errors:
    return;
}

/*******************************************************************/
void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain) {
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    if (power_domain_state[domain] == 0) goto errors;
    // Update state.
    power_domain_state[domain] &= ~(0b1 << requester_id);
    // Directly exit if this is not the last request.
    if (power_domain_state[domain] != 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_MCU_TCXO:
       // Turn MCU TCXO off.
       GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 0);
       break;
    case POWER_DOMAIN_ANALOG:
        // Release attached drivers and turn analog front-end off.
#ifdef HW1_0
        power_domain_state[POWER_DOMAIN_RADIO] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        power_domain_state[POWER_DOMAIN_SENSORS] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        // Check if power can be turned off.
        if ((power_domain_state[POWER_DOMAIN_RADIO] == 0) && (power_domain_state[POWER_DOMAIN_SENSORS] == 0)) {
            _POWER_analog_de_init();
            _POWER_radio_de_init();
            _POWER_sensors_de_init();
        }
#endif
#ifdef HW2_0
        _POWER_analog_de_init();
#endif
        break;
    case POWER_DOMAIN_SENSORS:
       // Release attached drivers and turn sensors off.
#ifdef HW1_0
        power_domain_state[POWER_DOMAIN_RADIO] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        power_domain_state[POWER_DOMAIN_ANALOG] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        // Check if power can be turned off.
        if ((power_domain_state[POWER_DOMAIN_RADIO] == 0) && (power_domain_state[POWER_DOMAIN_ANALOG] == 0)) {
            _POWER_analog_de_init();
            _POWER_radio_de_init();
            _POWER_sensors_de_init();
        }
#endif
#ifdef HW2_0
        _POWER_sensors_de_init();
#endif
        break;
    case POWER_DOMAIN_GPS:
        // Release attached drivers and turn GPS off.
        _POWER_gps_de_init();
        break;
    case POWER_DOMAIN_RADIO_TCXO:
        // Turn radio TCXO off.
        GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Release attached drivers and turn radio off.
#ifdef HW1_0
        power_domain_state[POWER_DOMAIN_SENSORS] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        power_domain_state[POWER_DOMAIN_ANALOG] &= ~(0b1 << POWER_REQUESTER_ID_POWER);
        // Check if power can be turned off.
        if ((power_domain_state[POWER_DOMAIN_SENSORS] == 0) && (power_domain_state[POWER_DOMAIN_ANALOG] == 0)) {
            _POWER_analog_de_init();
            _POWER_sensors_de_init();
            _POWER_radio_de_init();
        }
#endif
#ifdef HW2_0
        _POWER_radio_de_init();
#endif
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
errors:
    return;
}

/*******************************************************************/
uint8_t POWER_get_state(POWER_domain_t domain) {
    // Local variables.
    uint8_t state = 0;
    // Check parameters.
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    state = (power_domain_state[domain] == 0) ? 0 : 1;
errors:
    return state;
}
