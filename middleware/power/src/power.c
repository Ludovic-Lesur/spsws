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
#include "gpio.h"
#include "gpio_mapping.h"
#include "gps.h"
#include "lptim.h"
#include "rfe.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
#include "types.h"

/*** POWER local structures ***/

/*******************************************************************/
typedef struct {
    uint8_t state[POWER_DOMAIN_LAST];
#ifdef HW1_0
    uint8_t effective_state[POWER_DOMAIN_LAST];
#endif
} POWER_context_t;

/*** POWER local global variables ***/

static POWER_context_t power_ctx;

/*** POWER local functions ***/

/*******************************************************************/
static POWER_status_t _POWER_analog_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
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
    ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_ANALOG] = 1;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_analog_de_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    // Release internal and external ADC drivers.
    analog_status = ANALOG_de_init();
    ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
    // Turn external ADC off.
#ifdef HW1_0
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
#endif
#ifdef HW2_0
    GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
#endif
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_ANALOG] = 0;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_sensors_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    // Turn sensors on.
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
    // Init sensors drivers.
    dps310_status = DPS310_init();
    DPS310_exit_error(POWER_ERROR_BASE_DPS310);
    sht3x_status = SHT3X_init();
    SHT3X_exit_error(POWER_ERROR_BASE_SHT3X);
    si1133_status = SI1133_init();
    SI1133_exit_error(POWER_ERROR_BASE_SI1133);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_SENSORS] = 1;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_sensors_de_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    // Release sensors drivers.
    dps310_status = DPS310_de_init();
    DPS310_exit_error(POWER_ERROR_BASE_DPS310);
    sht3x_status = SHT3X_de_init();
    SHT3X_exit_error(POWER_ERROR_BASE_SHT3X);
    si1133_status = SI1133_de_init();
    SI1133_exit_error(POWER_ERROR_BASE_SI1133);
    // Turn sensors off.
    GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_SENSORS] = 0;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_gps_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    // Turn GPS on.
    GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
    // Init GPS driver.
    gps_status = GPS_init();
    GPS_exit_error(POWER_ERROR_BASE_GPS);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_GPS] = 1;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_gps_de_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    // Release GPS driver.
    gps_status = GPS_de_init();
    GPS_exit_error(POWER_ERROR_BASE_GPS);
    // Turn GPS off.
    GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_GPS] = 0;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_radio_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Turn radio on.
    GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
    // Init SX1232 and SKY13317 drivers.
    sx1232_status = SX1232_init();
    SX1232_exit_error(POWER_ERROR_BASE_SX1232);
    rfe_status = RFE_init();
    RFE_exit_error(POWER_ERROR_BASE_RFE);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_RADIO] = 1;
#endif
errors:
    return status;
}

/*******************************************************************/
static POWER_status_t _POWER_radio_de_init(void) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Release SX1232 and SKY13317 drivers.
    sx1232_status = SX1232_de_init();
    SX1232_exit_error(POWER_ERROR_BASE_SX1232);
    rfe_status = RFE_de_init();
    RFE_exit_error(POWER_ERROR_BASE_RFE);
    // Turn radio off.
    GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
#ifdef HW1_0
    power_ctx.effective_state[POWER_DOMAIN_RADIO] = 0;
#endif
errors:
    return status;
}

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
    for (domain = 0; domain < POWER_DOMAIN_LAST; domain++) {
        POWER_disable(domain);
    }
}

/*******************************************************************/
POWER_status_t POWER_enable(POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint32_t delay_ms = 0;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
#ifdef HW1_0
        if (power_ctx.effective_state[POWER_DOMAIN_RADIO] == 0) {
            status = _POWER_radio_init();
            if (status != POWER_SUCCESS) goto errors;
        }
        if (power_ctx.effective_state[POWER_DOMAIN_SENSORS] == 0) {
            status = _POWER_sensors_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
        status = _POWER_analog_init();
        if (status != POWER_SUCCESS) goto errors;
        // Update delay.
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
        break;
    case POWER_DOMAIN_SENSORS:
#ifdef HW1_0
        if (power_ctx.effective_state[POWER_DOMAIN_ANALOG] == 0) {
            status = _POWER_analog_init();
            if (status != POWER_SUCCESS) goto errors;
        }
        if (power_ctx.effective_state[POWER_DOMAIN_RADIO] == 0) {
            status = _POWER_radio_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
        status = _POWER_sensors_init();
        if (status != POWER_SUCCESS) goto errors;
        // Update delay.
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        break;
    case POWER_DOMAIN_GPS:
        status = _POWER_gps_init();
        if (status != POWER_SUCCESS) goto errors;
        // Update delay.
        delay_ms = POWER_ON_DELAY_MS_GPS;
        break;
    case POWER_DOMAIN_RADIO_TCXO:
        // Turn radio TCXO on.
        GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 1);
        // Update delay.
        delay_ms = POWER_ON_DELAY_MS_RADIO_TCXO;
        break;
    case POWER_DOMAIN_RADIO:
#ifdef HW1_0
        if (power_ctx.effective_state[POWER_DOMAIN_ANALOG] == 0) {
            GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
            status = _POWER_analog_init();
            if (status != POWER_SUCCESS) goto errors;
        }
        if (power_ctx.effective_state[POWER_DOMAIN_SENSORS] == 0) {
            status = _POWER_sensors_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
        status = _POWER_radio_init();
        if (status != POWER_SUCCESS) goto errors;
        // Update delay.
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        break;
    default:/*******************************************************************/
        status = POWER_ERROR_DOMAIN;
        goto errors;
    }
    // Update state.
    power_ctx.state[domain] = 1;
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        LPTIM_exit_error(POWER_ERROR_BASE_LPTIM);
    }
errors:
    return status;
}

/*******************************************************************/
POWER_status_t POWER_disable(POWER_domain_t domain) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
#ifdef HW1_0
        if ((power_ctx.state[POWER_DOMAIN_RADIO] == 0) && (power_ctx.state[POWER_DOMAIN_SENSORS] == 0)) {
            status = _POWER_analog_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_radio_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_sensors_de_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
#ifdef HW2_0
        status = _POWER_analog_de_init();
        if (status != POWER_SUCCESS) goto errors;
#endif
        break;
    case POWER_DOMAIN_SENSORS:
#ifdef HW1_0
        if ((power_ctx.state[POWER_DOMAIN_RADIO] == 0) && (power_ctx.state[POWER_DOMAIN_ANALOG] == 0)) {
            status = _POWER_analog_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_radio_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_sensors_de_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
#ifdef HW2_0
        status = _POWER_sensors_de_init();
        if (status != POWER_SUCCESS) goto errors;
#endif
        break;
    case POWER_DOMAIN_GPS:
        status = _POWER_gps_de_init();
        if (status != POWER_SUCCESS) goto errors;
        break;
    case POWER_DOMAIN_RADIO_TCXO:
        // Turn radio TCXO off.
        GPIO_write(&GPIO_TCXO32_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Turn radio off.
#ifdef HW1_0
        if ((power_ctx.state[POWER_DOMAIN_SENSORS] == 0) && (power_ctx.state[POWER_DOMAIN_ANALOG] == 0)) {
            status = _POWER_analog_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_sensors_de_init();
            if (status != POWER_SUCCESS) goto errors;
            status = _POWER_radio_de_init();
            if (status != POWER_SUCCESS) goto errors;
        }
#endif
#ifdef HW2_0
        status = _POWER_radio_de_init();
        if (status != POWER_SUCCESS) goto errors;
#endif
        break;
    default:
        status = POWER_ERROR_DOMAIN;
        goto errors;
    }
    // Update state.
    power_ctx.state[domain] = 0;
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
    (*state) = power_ctx.state[domain];
errors:
    return status;
}
