/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#include "at.h"

// Peripherals.
#include "gpio_mapping.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "pwr.h"
#include "rcc.h"
#include "usart.h"
// Utils.
#include "error.h"
#include "math.h"
#include "parser.h"
#include "string.h"
#include "types.h"
// Components.
#include "dps310.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
// Middleware.
#include "analog.h"
#include "gps.h"
#include "power.h"
#include "rfe.h"
// Sigfox.
#include "manuf/rf_api.h"
#include "sigfox_ep_addon_rfp_api.h"
#include "sigfox_ep_api.h"
#include "sigfox_error.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
// Applicative.
#include "error_base.h"
#include "mode.h"
#include "version.h"

/*** AT local macros ***/

// USART.
#ifdef HW1_0
#define AT_USART_INSTANCE               USART_INSTANCE_USART2
#endif
#ifdef HW2_0
#define AT_USART_INSTANCE               USART_INSTANCE_USART1
#endif
#define AT_USART_BAUD_RATE              9600
// Commands.
#define AT_COMMAND_BUFFER_SIZE          128
// Parameters separator.
#define AT_CHAR_SEPARATOR               ','
// Replies.
#define AT_REPLY_BUFFER_SIZE            128
#define AT_REPLY_END                    "\r\n"
#define AT_REPLY_TAB                    "     "
#define AT_STRING_VALUE_BUFFER_SIZE     16
// Duration of RSSI command.
#define AT_RSSI_REPORT_PERIOD_MS        500
// Enabled commands.
#define AT_COMMAND_NVM
#define AT_COMMAND_SENSORS
#define AT_COMMAND_GPS
#define AT_COMMAND_SIGFOX_EP_LIB
#define AT_COMMAND_SIGFOX_EP_ADDON_RFP
#define AT_COMMAND_CW
#define AT_COMMAND_RSSI

/*** AT callbacks declaration ***/

#ifdef ATM
/*******************************************************************/
static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_print_sw_version(void);
static void _AT_print_error_stack(void);
static void _AT_rcc_callback(void);
/*******************************************************************/
#ifdef AT_COMMAND_NVM
static void _AT_nvm_callback(void);
static void _AT_get_id_callback(void);
static void _AT_set_id_callback(void);
static void _AT_get_key_callback(void);
static void _AT_set_key_callback(void);
#endif
/*******************************************************************/
#ifdef AT_COMMAND_SENSORS
static void _AT_adc_callback(void);
static void _AT_iths_callback(void);
#ifdef HW2_0
static void _AT_eths_callback(void);
#endif
static void _AT_epts_callback(void);
static void _AT_euvs_callback(void);
#endif
/*******************************************************************/
#ifdef AT_COMMAND_GPS
static void _AT_time_callback(void);
static void _AT_gps_callback(void);
#endif
/*******************************************************************/
#ifdef AT_COMMAND_SIGFOX_EP_LIB
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
static void _AT_so_callback(void);
#endif
static void _AT_sb_callback(void);
static void _AT_sf_callback(void);
#endif
/*******************************************************************/
#ifdef AT_COMMAND_SIGFOX_EP_ADDON_RFP
static void _AT_tm_callback(void);
#endif
/*******************************************************************/
#ifdef AT_COMMAND_CW
static void _AT_cw_callback(void);
#endif
#if (defined AT_COMMAND_RSSI) && (defined BIDIRECTIONAL)
static void _AT_rssi_callback(void);
#endif
#endif

/*** AT local structures ***/

#ifdef ATM
/*******************************************************************/
typedef struct {
    PARSER_mode_t mode;
    char_t* syntax;
    char_t* parameters;
    char_t* description;
    void (*callback)(void);
} AT_command_t;
#endif

#ifdef ATM
/*******************************************************************/
typedef struct {
    // Command.
    volatile char_t command[AT_COMMAND_BUFFER_SIZE];
    volatile uint32_t command_size;
    volatile uint8_t line_end_flag;
    PARSER_context_t parser;
    // Reply.
    char_t reply[AT_REPLY_BUFFER_SIZE];
    uint32_t reply_size;
} AT_context_t;
#endif

/*** AT local global variables ***/

#ifdef ATM
// @formatter:off
static const AT_command_t AT_COMMAND_LIST[] = {
    { PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_print_ok },
    { PARSER_MODE_COMMAND, "AT?", STRING_NULL, "AT commands list", _AT_print_command_list },
    { PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_print_sw_version },
    { PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_print_error_stack },
    { PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset },
    { PARSER_MODE_COMMAND, "AT$RCC?", STRING_NULL, "Get clocks frequency", _AT_rcc_callback },
#ifdef AT_COMMAND_NVM
    { PARSER_MODE_HEADER, "AT$NVM=", "address[dec]", "Get NVM data", _AT_nvm_callback },
    { PARSER_MODE_COMMAND, "AT$ID?", STRING_NULL, "Get Sigfox EP ID", _AT_get_id_callback },
    { PARSER_MODE_HEADER, "AT$ID=", "id[hex]", "Set Sigfox EP ID", _AT_set_id_callback },
    { PARSER_MODE_COMMAND, "AT$KEY?", STRING_NULL, "Get Sigfox EP key", _AT_get_key_callback },
    { PARSER_MODE_HEADER, "AT$KEY=", "key[hex]", "Set Sigfox EP key", _AT_set_key_callback },
#endif
#ifdef AT_COMMAND_SENSORS
    { PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get analog measurements", _AT_adc_callback },
    { PARSER_MODE_COMMAND, "AT$ITHS?", STRING_NULL, "Get internal PCB temperature and humidity", _AT_iths_callback },
#ifdef HW2_0
    { PARSER_MODE_COMMAND, "AT$ETHS?", STRING_NULL, "Get external temperature and humidity", _AT_eths_callback },
#endif
    { PARSER_MODE_COMMAND, "AT$EPTS?", STRING_NULL, "Get pressure and temperature", _AT_epts_callback },
    { PARSER_MODE_COMMAND, "AT$EUVS?", STRING_NULL, "Get UV index", _AT_euvs_callback },
#endif
#ifdef AT_COMMAND_GPS
    { PARSER_MODE_HEADER, "AT$TIME=", "timeout[s]", "Get GPS time", _AT_time_callback },
    { PARSER_MODE_HEADER, "AT$GPS=", "timeout[s]", "Get GPS position", _AT_gps_callback },
#endif
#ifdef AT_COMMAND_SIGFOX_EP_LIB
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
    { PARSER_MODE_COMMAND, "AT$SO", STRING_NULL, "Sigfox send control message", _AT_so_callback },
#endif
    { PARSER_MODE_HEADER, "AT$SB=", "data[bit],(bidir_flag[bit])", "Sigfox send bit", _AT_sb_callback },
    { PARSER_MODE_HEADER, "AT$SF=", "data[hex],(bidir_flag[bit])", "Sigfox send frame", _AT_sf_callback },
#endif
#ifdef AT_COMMAND_SIGFOX_EP_ADDON_RFP
    { PARSER_MODE_HEADER, "AT$TM=", "bit_rate_index[dec],test_mode_reference[dec]", "Sigfox RFP test mode", _AT_tm_callback },
#endif
#ifdef AT_COMMAND_CW
    { PARSER_MODE_HEADER, "AT$CW=", "frequency[hz],enable[bit],(output_power[dbm])", "Continuous wave", _AT_cw_callback },
#endif
#if (defined AT_COMMAND_RSSI) && (defined BIDIRECTIONAL)
    { PARSER_MODE_HEADER, "AT$RSSI=", "frequency[hz],duration[s]", "Continuous RSSI measurement", _AT_rssi_callback },
#endif
};
// @formatter:on
static AT_context_t at_ctx;
#endif

/*** AT local functions ***/

/*******************************************************************/
#define _AT_sigfox_ep_addon_rfp_stack_exit_error(void) { \
    if (sigfox_ep_addon_rfp_status != SIGFOX_EP_ADDON_RFP_API_SUCCESS) { \
        status = ERROR_BASE_SIGFOX_EP_ADDON_RFP + sigfox_ep_addon_rfp_status; \
        ERROR_stack_add(status); \
        goto errors; \
    } \
}

#ifdef ATM
/*******************************************************************/
#define _AT_reply_add_char(character) { \
    at_ctx.reply[at_ctx.reply_size] = character; \
    at_ctx.reply_size = (at_ctx.reply_size + 1) % AT_REPLY_BUFFER_SIZE; \
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_fill_rx_buffer(uint8_t rx_byte) {
    // Append byte if line end flag is not already set.
    if (at_ctx.line_end_flag == 0) {
        // Check ending characters.
        if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
            at_ctx.command[at_ctx.command_size] = STRING_CHAR_NULL;
            at_ctx.line_end_flag = 1;
        }
        else {
            // Store new byte.
            at_ctx.command[at_ctx.command_size] = rx_byte;
            // Manage index.
            at_ctx.command_size = (at_ctx.command_size + 1) % AT_COMMAND_BUFFER_SIZE;
        }
    }
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_reply_add_string(char_t* tx_string) {
    // Fill TX buffer with new bytes.
    while (*tx_string) {
        _AT_reply_add_char(*(tx_string++));
        // Detect rollover.
        if (at_ctx.reply_size == 0) break;
    }
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
    // Local variables.
    STRING_status_t string_status = STRING_SUCCESS;
    char_t str_value[AT_STRING_VALUE_BUFFER_SIZE];
    uint8_t idx = 0;
    // Reset string.
    for (idx = 0; idx < AT_STRING_VALUE_BUFFER_SIZE; idx++)
        str_value[idx] = STRING_CHAR_NULL;
    // Convert value to string.
    string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
    STRING_stack_error(ERROR_BASE_STRING);
    // Add string.
    _AT_reply_add_string(str_value);
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_reply_send(void) {
    // Local variables.
    USART_status_t usart_status = USART_SUCCESS;
    // Add ending string.
    _AT_reply_add_string(AT_REPLY_END);
    // Send response over UART.
    usart_status = USART_write(AT_USART_INSTANCE, (uint8_t*) at_ctx.reply, at_ctx.reply_size);
    USART_stack_error(ERROR_BASE_USART_AT);
    // Flush reply buffer.
    at_ctx.reply_size = 0;
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_print_ok(void) {
    _AT_reply_add_string("OK");
    _AT_reply_send();
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_print_error(ERROR_code_t error) {
    // Print error.
    _AT_reply_add_string("ERROR_");
    if (error < 0x0100) {
        _AT_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
        _AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
    }
    else {
        _AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
    }
    _AT_reply_send();
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_print_command_list(void) {
    // Local variables.
    uint32_t idx = 0;
    // Commands loop.
    for (idx = 0; idx < (sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)); idx++) {
        // Print syntax.
        _AT_reply_add_string(AT_COMMAND_LIST[idx].syntax);
        // Print parameters.
        _AT_reply_add_string(AT_COMMAND_LIST[idx].parameters);
        _AT_reply_send();
        // Print description.
        _AT_reply_add_string(AT_REPLY_TAB);
        _AT_reply_add_string(AT_COMMAND_LIST[idx].description);
        _AT_reply_send();
    }
    _AT_print_ok();
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_print_sw_version(void) {
    _AT_reply_add_string("sw");
    _AT_reply_add_value((int32_t) GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string(".");
    _AT_reply_add_value((int32_t) GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string(".");
    _AT_reply_add_value((int32_t) GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
    if (GIT_DIRTY_FLAG != 0) {
        _AT_reply_add_string(".d");
    }
    _AT_reply_add_string(" (");
    _AT_reply_add_value((int32_t) GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
    _AT_reply_add_string(")");
    _AT_reply_send();
    _AT_print_ok();
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_print_error_stack(void) {
    // Local variables.
    ERROR_code_t error = SUCCESS;
    // Import Sigfox errors into MCU stack.
    ERROR_import_sigfox_stack();
    // Unstack all errors.
    _AT_reply_add_string("[ ");
    do {
        error = ERROR_stack_read();
        if (error != SUCCESS) {
            _AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
            _AT_reply_add_string(" ");
        }
    }
    while (error != SUCCESS);
    _AT_reply_add_string("]");
    _AT_reply_send();
    _AT_print_ok();
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_rcc_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    char_t* rcc_clock_name[RCC_CLOCK_LAST] = { "LSI", "LSE", "MSI", "HSI", "SYS" };
    uint8_t clock_status = 0;
    uint32_t clock_frequency = 0;
    uint8_t idx = 0;
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_exit_error(ERROR_BASE_RCC, (ERROR_BASE_RCC + rcc_status));
    // Clocks loop.
    for (idx = 0; idx < RCC_CLOCK_LAST; idx++) {
        // Read status.
        rcc_status = RCC_get_status(idx, &clock_status);
        RCC_stack_exit_error(ERROR_BASE_RCC, (ERROR_BASE_RCC + rcc_status));
        // Read frequency.
        rcc_status = RCC_get_frequency_hz(idx, &clock_frequency);
        RCC_stack_exit_error(ERROR_BASE_RCC, (ERROR_BASE_RCC + rcc_status));
        // Print data.
        _AT_reply_add_string(rcc_clock_name[idx]);
        _AT_reply_add_string((clock_status == 0) ? ": OFF " : ": ON  ");
        _AT_reply_add_value((int32_t) clock_frequency, STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("Hz");
        _AT_reply_send();
    }
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_NVM)
/*******************************************************************/
static void _AT_nvm_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    NVM_status_t nvm_status = NVM_SUCCESS;
    int32_t address = 0;
    uint8_t nvm_data = 0;
    // Read address parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &address);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Read byte at requested address.
    nvm_status = NVM_read_byte((NVM_address_t) address, &nvm_data);
    NVM_stack_exit_error(ERROR_BASE_NVM, (ERROR_BASE_NVM + nvm_status));
    // Print data.
    _AT_reply_add_value((int32_t) nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_NVM)
/*******************************************************************/
static void _AT_get_id_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    uint8_t id_byte = 0;
    // Retrieve device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + idx), &id_byte);
        NVM_stack_exit_error(ERROR_BASE_NVM, (ERROR_BASE_NVM + nvm_status));
        _AT_reply_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, ((idx == 0) ? 1 : 0));
    }
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_NVM)
/*******************************************************************/
static void _AT_set_id_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t sigfox_ep_id[SIGFOX_EP_ID_SIZE_BYTES];
    uint8_t extracted_length = 0;
    uint8_t idx = 0;
    // Read ID parameter.
    parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, SIGFOX_EP_ID_SIZE_BYTES, 1, sigfox_ep_id, &extracted_length);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Write device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_ID + idx), sigfox_ep_id[idx]);
        NVM_stack_exit_error(ERROR_BASE_NVM, (ERROR_BASE_NVM + nvm_status));
    }
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_NVM)
/*******************************************************************/
static void _AT_get_key_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    uint8_t key_byte = 0;
    // Retrieve device key in NVM.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), &key_byte);
        NVM_stack_exit_error(ERROR_BASE_NVM, (ERROR_BASE_NVM + nvm_status));
        _AT_reply_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, ((idx == 0) ? 1 : 0));
    }
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_NVM)
/*******************************************************************/
static void _AT_set_key_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t sigfox_ep_key[SIGFOX_EP_KEY_SIZE_BYTES];
    uint8_t extracted_length = 0;
    uint8_t idx = 0;
    // Read key parameter.
    parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, SIGFOX_EP_KEY_SIZE_BYTES, 1, sigfox_ep_key, &extracted_length);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Write device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), sigfox_ep_key[idx]);
        NVM_stack_exit_error(ERROR_BASE_NVM, (ERROR_BASE_NVM + nvm_status));
    }
    _AT_print_ok();
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SENSORS)
/*******************************************************************/
static void _AT_adc_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t generic_s32 = 0;
    // Turn analog front-end on.
    power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // MCU voltage.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VMCU_MV, &generic_s32);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (ERROR_BASE_ANALOG + analog_status));
    _AT_reply_add_string("Vmcu=");
    _AT_reply_add_value(generic_s32, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("mV");
    _AT_reply_send();
    // MCU temperature.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_TMCU_DEGREES, &generic_s32);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (ERROR_BASE_ANALOG + analog_status));
    _AT_reply_add_string("Tmcu=");
    _AT_reply_add_value(generic_s32, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("dC");
    _AT_reply_send();
    // Source voltage.
    _AT_reply_add_string("Vpv=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VPV_MV, &generic_s32);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (ERROR_BASE_ANALOG + analog_status));
    _AT_reply_add_value(generic_s32, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("mV");
    _AT_reply_send();
    // Supercap voltage.
    _AT_reply_add_string("Vcap=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VCAP_MV, &generic_s32);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (ERROR_BASE_ANALOG + analog_status));
    _AT_reply_add_value(generic_s32, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("mV");
    _AT_reply_send();
    // Light.
    _AT_reply_add_string("Light=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_LDR_PERCENT, &generic_s32);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (ERROR_BASE_ANALOG + analog_status));
    _AT_reply_add_value(generic_s32, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("%");
    _AT_reply_send();
    // Turn analog front-end off.
    power_status = POWER_disable(POWER_DOMAIN_ANALOG);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_ANALOG);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SENSORS)
/*******************************************************************/
static void _AT_iths_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    int32_t temperature_degrees = 0;
    int32_t humidity_percent = 0;
    // Turn digital sensors on.
    power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform measurements.
    sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_INTERNAL, &temperature_degrees, &humidity_percent);
    SHT3X_stack_exit_error(ERROR_BASE_SHT30_INTERNAL, (ERROR_BASE_SHT30_INTERNAL + sht3x_status));
    // Turn digital sensors off.
    power_status = POWER_disable(POWER_DOMAIN_SENSORS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Read and print data.
    // Temperature.
    _AT_reply_add_string("Tpcb=");
    _AT_reply_add_value(temperature_degrees, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("dC");
    _AT_reply_send();
    // Humidity.
    _AT_reply_add_string("Hpcb=");
    _AT_reply_add_value(humidity_percent, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("%");
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_SENSORS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SENSORS) && (defined HW2_0)
/*******************************************************************/
static void _AT_eths_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    int32_t temperature_degrees = 0;
    int32_t humidity_percent = 0;
    // Turn digital sensors on.
    power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform measurements.
    sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_EXTERNAL, &temperature_degrees, &humidity_percent);
    SHT3X_stack_exit_error(ERROR_BASE_SHT30_EXTERNAL, (ERROR_BASE_SHT30_EXTERNAL + sht3x_status));
    // Turn digital sensors off.
    power_status = POWER_disable(POWER_DOMAIN_SENSORS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Read and print data.
    // Temperature.
    _AT_reply_add_string("Tamb=");
    _AT_reply_add_value(temperature_degrees, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("dC");
    _AT_reply_send();
    // Humidity.
    _AT_reply_add_string("Hamb=");
    _AT_reply_add_value(humidity_percent, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("%");
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_SENSORS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SENSORS)
/*******************************************************************/
static void _AT_epts_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    int32_t pressure_pa = 0;
    int32_t temperature_degrees = 0;
    // Turn digital sensors on.
    power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform measurements.
    dps310_status = DPS310_get_pressure_temperature(I2C_ADDRESS_DPS310, &pressure_pa, &temperature_degrees);
    DPS310_stack_exit_error(ERROR_BASE_DPS310, (ERROR_BASE_DPS310 + dps310_status));
    // Turn digital sensors off.
    power_status = POWER_disable(POWER_DOMAIN_SENSORS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Read and print data.
    // Pressure.
    _AT_reply_add_string("Pabs=");
    _AT_reply_add_value(pressure_pa, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("Pa");
    _AT_reply_send();
    // Temperature.
    _AT_reply_add_string("Tamb=");
    _AT_reply_add_value(temperature_degrees, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("dC");
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_SENSORS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SENSORS)
/*******************************************************************/
static void _AT_euvs_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    int32_t uv_index = 0;
    // Turn digital sensors on.
    power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform measurements.
    si1133_status = SI1133_get_uv_index(I2C_ADDRESS_SI1133, &uv_index);
    SI1133_stack_exit_error(ERROR_BASE_SI1133, (ERROR_BASE_SI1133 + si1133_status));
    // Turn digital sensors off.
    power_status = POWER_disable(POWER_DOMAIN_SENSORS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Read and print data.
    _AT_reply_add_string("UVI=");
    _AT_reply_add_value(uv_index, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_SENSORS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_GPS)
/*******************************************************************/
static void _AT_time_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    GPS_status_t gps_status = GPS_SUCCESS;
    GPS_acquisition_status_t acquisition_status = GPS_ACQUISITION_ERROR_LAST;
    GPS_time_t gps_time;
    int32_t timeout_seconds = 0;
    uint32_t fix_duration_seconds = 0;
    // Read timeout parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Turn GPS on.
    power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform time acquisition.
    gps_status = GPS_get_time(&gps_time, (uint32_t) timeout_seconds, &fix_duration_seconds, &acquisition_status);
    GPS_stack_exit_error(ERROR_BASE_GPS, ERROR_BASE_GPS + gps_status);
    // Turn GPS off.
    power_status = POWER_disable(POWER_DOMAIN_GPS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Check status.
    if (acquisition_status == GPS_ACQUISITION_SUCCESS) {
        // Year.
        _AT_reply_add_value((int32_t) (gps_time.year), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("-");
        // Month.
        if ((gps_time.month) < 10) {
            _AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
        }
        _AT_reply_add_value((int32_t) (gps_time.month), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("-");
        // Day.
        if ((gps_time.date) < 10) {
            _AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
        }
        _AT_reply_add_value((int32_t) (gps_time.date), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string(" ");
        // Hours.
        if ((gps_time.hours) < 10) {
            _AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
        }
        _AT_reply_add_value((int32_t) (gps_time.hours), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string(":");
        // Minutes.
        if ((gps_time.minutes) < 10) {
            _AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
        }
        _AT_reply_add_value((int32_t) (gps_time.minutes), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string(":");
        // Seconds.
        if ((gps_time.seconds) < 10) {
            _AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
        }
        _AT_reply_add_value((int32_t) (gps_time.seconds), STRING_FORMAT_DECIMAL, 0);
    }
    else {
        _AT_reply_add_string("GPS timeout");
    }
    _AT_reply_send();
    _AT_print_ok();
    return;
errors:
    POWER_disable(POWER_DOMAIN_GPS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_GPS)
/*******************************************************************/
static void _AT_gps_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    GPS_status_t gps_status = GPS_SUCCESS;
    GPS_position_t gps_position;
    GPS_acquisition_status_t acquisition_status = GPS_ACQUISITION_ERROR_LAST;
    int32_t timeout_seconds = 0;
    uint32_t fix_duration_seconds = 0;
    // Read timeout parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Turn GPS on.
    power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    // Perform time acquisition.
    gps_status = GPS_get_position(&gps_position, (uint32_t) timeout_seconds, &fix_duration_seconds, &acquisition_status);
    GPS_stack_exit_error(ERROR_BASE_GPS, ERROR_BASE_GPS + gps_status);
    // Check status.
    if (acquisition_status == GPS_ACQUISITION_SUCCESS) {
        // Latitude.
        _AT_reply_add_string("Lat=");
        _AT_reply_add_value((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("d");
        _AT_reply_add_value((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("'");
        _AT_reply_add_value((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("''");
        _AT_reply_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
        // Longitude.
        _AT_reply_add_string(" Long=");
        _AT_reply_add_value((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("d");
        _AT_reply_add_value((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("'");
        _AT_reply_add_value((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("''");
        _AT_reply_add_string(((gps_position.long_east_flag) == 0) ? "W" : "E");
        // Altitude.
        _AT_reply_add_string(" Alt=");
        _AT_reply_add_value((gps_position.altitude), STRING_FORMAT_DECIMAL, 0);
        // Fix duration.
        _AT_reply_add_string("m Fix=");
        _AT_reply_add_value(fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("s");
    }
    else {
        _AT_reply_add_string("GPS timeout");
    }
    _AT_reply_send();
    _AT_print_ok();
    // Turn GPS off.
    power_status = POWER_disable(POWER_DOMAIN_GPS);
    POWER_stack_exit_error(ERROR_BASE_POWER, ERROR_BASE_POWER + power_status);
    return;
errors:
    POWER_disable(POWER_DOMAIN_GPS);
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SIGFOX_EP_LIB) && (defined BIDIRECTIONAL)
/*******************************************************************/
static void _AT_print_dl_payload(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm = 0;
    // Read downlink payload.
    sigfox_ep_api_status = SIGFOX_EP_API_get_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi_dbm);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    // Print downlink payload.
    AT_print_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, dl_rssi_dbm);
    return;
errors:
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SIGFOX_EP_LIB) && (defined CONTROL_KEEP_ALIVE_MESSAGE)
/*******************************************************************/
static void _AT_so_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_control_message_t control_message;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default control message parameters.
    control_message.type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
    control_message.common_parameters.number_of_frames = 3;
    control_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_control_message(&control_message);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    _AT_print_ok();
    return;
errors:
    // Close library.
    SIGFOX_EP_API_close();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SIGFOX_EP_LIB)
/*******************************************************************/
static void _AT_sb_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_application_message_t application_message;
    int32_t ul_bit = 0;
    int32_t bidir_flag = 0;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default application message parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.ul_payload = SFX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // First try with 2 parameters.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &ul_bit);
    if (parser_status == PARSER_SUCCESS) {
        // Try parsing downlink request parameter.
        parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update parameters.
        application_message.type = (SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0 + ul_bit);
#ifdef BIDIRECTIONAL
        application_message.bidirectional_flag = bidir_flag;
#endif
    }
    else {
        // Try with 1 parameter.
        parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &ul_bit);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update parameters.
        application_message.type = (SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0 + ul_bit);
#ifdef BIDIRECTIONAL
        application_message.bidirectional_flag = 0;
#endif
    }
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&application_message);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
#ifdef BIDIRECTIONAL
    // Read and print DL payload if needed.
    if ((application_message.bidirectional_flag) == SFX_TRUE) {
        _AT_print_dl_payload();
    }
#endif
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    _AT_print_ok();
    return;
errors:
    // Close library.
    SIGFOX_EP_API_close();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SIGFOX_EP_LIB)
/*******************************************************************/
static void _AT_sf_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_application_message_t application_message;
    sfx_u8 data[SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES];
    uint8_t extracted_length = 0;
    int32_t bidir_flag = 0;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default application message parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef BIDIRECTIONAL
    application_message.bidirectional_flag = 0;
#endif
    application_message.ul_payload = SFX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // First try with 2 parameters.
    parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES, 0, data, &extracted_length);
    if (parser_status == PARSER_SUCCESS) {
        // Try parsing downlink request parameter.
        parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update parameters.
        application_message.ul_payload = (sfx_u8*) data;
        application_message.ul_payload_size_bytes = extracted_length;
#ifdef BIDIRECTIONAL
        application_message.bidirectional_flag = bidir_flag;
#endif
    }
    else {
        // Try with 1 parameter.
        parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES, 0, data, &extracted_length);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update parameters.
        application_message.ul_payload = (sfx_u8*) data;
        application_message.ul_payload_size_bytes = extracted_length;
    }
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&application_message);
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
#ifdef BIDIRECTIONAL
    // Read and print DL payload if needed.
    if ((application_message.bidirectional_flag) == SFX_TRUE) {
        _AT_print_dl_payload();
    }
#endif
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * 0x0100) + sigfox_ep_api_status);
    _AT_print_ok();
    return;
errors:
    // Close library.
    SIGFOX_EP_API_close();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_SIGFOX_EP_ADDON_RFP)
/*******************************************************************/
static void _AT_tm_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    SIGFOX_EP_ADDON_RFP_API_status_t sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_SUCCESS;
    SIGFOX_EP_ADDON_RFP_API_config_t addon_config;
    SIGFOX_EP_ADDON_RFP_API_test_mode_t test_mode;
    int32_t bit_rate_index = 0;
    int32_t test_mode_reference = 0;
    // Read bit rate.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &bit_rate_index);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Read test mode parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &test_mode_reference);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Addon configuration.
    addon_config.rc = &SIGFOX_RC1;
    // Test mode parameters.
    test_mode.test_mode_reference = (SIGFOX_EP_ADDON_RFP_API_test_mode_reference_t) test_mode_reference;
    test_mode.ul_bit_rate = (SIGFOX_ul_bit_rate_t) bit_rate_index;
    // Open addon.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_open(&addon_config);
    _AT_sigfox_ep_addon_rfp_stack_exit_error();
    // Call test mode function.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_test_mode(&test_mode);
    _AT_sigfox_ep_addon_rfp_stack_exit_error();
    // Close addon.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_close();
    _AT_sigfox_ep_addon_rfp_stack_exit_error();
    _AT_print_ok();
    return;
errors:
    // Close addon.
    SIGFOX_EP_ADDON_RFP_API_close();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_CW)
/*******************************************************************/
static void _AT_cw_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    RF_API_radio_parameters_t radio_params;
    int32_t enable = 0;
    int32_t frequency_hz = 0;
    int32_t power_dbm = 0;
    // Set common radio parameters.
    radio_params.rf_mode = RF_API_MODE_TX;
    radio_params.modulation = RF_API_MODULATION_NONE;
    radio_params.bit_rate_bps = 0;
#ifdef BIDIRECTIONAL
    radio_params.deviation_hz = 0;
#endif
    // Read frequency parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Update radio configuration.
    radio_params.frequency_hz = (sfx_u32) frequency_hz;
    // First try with 3 parameters.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &enable);
    if (parser_status == PARSER_SUCCESS) {
        // There is a third parameter, try to parse power.
        parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &power_dbm);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update radio configuration.
        radio_params.tx_power_dbm_eirp = (sfx_s8) power_dbm;
    }
    else {
        // Power is not given, try to parse enable as last parameter.
        parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
        PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
        // Update radio configuration.
        radio_params.tx_power_dbm_eirp = TX_POWER_DBM_EIRP;
    }
    // Stop CW.
    rf_api_status = RF_API_de_init();
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    rf_api_status = RF_API_sleep();
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    // Restart if required.
    if (enable != 0) {
        // Init radio.
        rf_api_status = RF_API_wake_up();
        RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
        rf_api_status = RF_API_init(&radio_params);
        RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
        // Start CW.
        rf_api_status = RF_API_start_continuous_wave();
        RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
        _AT_reply_add_string("CW running...");
        _AT_reply_send();
    }
    _AT_print_ok();
    return;
errors:
    // Force radio off.
    RF_API_de_init();
    RF_API_sleep();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#if (defined ATM) && (defined AT_COMMAND_RSSI) && (defined BIDIRECTIONAL)
/*******************************************************************/
static void _AT_rssi_callback(void) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    RF_API_radio_parameters_t radio_params;
    int32_t frequency_hz = 0;
    int32_t duration_seconds = 0;
    int16_t rssi_dbm = 0;
    uint32_t report_loop = 0;
    // Read frequency parameter.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Read duration parameters.
    parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &duration_seconds);
    PARSER_stack_exit_error(ERROR_BASE_PARSER, (ERROR_BASE_PARSER + parser_status));
    // Radio configuration.
    radio_params.rf_mode = RF_API_MODE_RX;
    radio_params.frequency_hz = (sfx_u32) frequency_hz;
    radio_params.modulation = RF_API_MODULATION_NONE;
    radio_params.bit_rate_bps = 0;
    radio_params.tx_power_dbm_eirp = TX_POWER_DBM_EIRP;
    radio_params.deviation_hz = 0;
    // Init radio.
    rf_api_status = RF_API_wake_up();
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    rf_api_status = RF_API_init(&radio_params);
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    // Start continuous listening.
    sx1232_status = SX1232_start_rx();
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (ERROR_BASE_SX1232 + sx1232_status));
    // Measurement loop.
    while (report_loop < ((uint32_t) ((duration_seconds * 1000) / AT_RSSI_REPORT_PERIOD_MS))) {
        // Read RSSI.
        rfe_status = RFE_get_rssi(&rssi_dbm);
        RFE_stack_exit_error(ERROR_BASE_RFE, (ERROR_BASE_RFE + rfe_status));
        // Print RSSI.
        _AT_reply_add_string("RSSI=");
        _AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
        _AT_reply_add_string("dBm");
        _AT_reply_send();
        // Report delay.
        lptim_status = LPTIM_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, LPTIM_DELAY_MODE_ACTIVE);
        LPTIM_stack_exit_error(ERROR_BASE_LPTIM, (ERROR_BASE_LPTIM + lptim_status));
        report_loop++;
        // Reload watchdog.
        IWDG_reload();
    }
    // Turn radio off.
    rf_api_status = RF_API_de_init();
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    rf_api_status = RF_API_sleep();
    RF_API_check_status(ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * 0x0100) + rf_api_status);
    _AT_print_ok();
    return;
errors:
    // Force radio off.
    RF_API_de_init();
    RF_API_sleep();
    // Print error.
    _AT_print_error(status);
    return;
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_reset_parser(void) {
    // Flush buffers.
    at_ctx.command_size = 0;
    at_ctx.reply_size = 0;
    // Reset flag.
    at_ctx.line_end_flag = 0;
    // Reset parser.
    at_ctx.parser.buffer = (char_t*) at_ctx.command;
    at_ctx.parser.buffer_size = 0;
    at_ctx.parser.separator_idx = 0;
    at_ctx.parser.start_idx = 0;
}
#endif

#ifdef ATM
/*******************************************************************/
static void _AT_decode(void) {
    // Local variables.
    uint8_t idx = 0;
    uint8_t decode_success = 0;
    // Update parser length.
    at_ctx.parser.buffer_size = at_ctx.command_size;
    // Loop on available commands.
    for (idx = 0; idx < (sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)); idx++) {
        // Check type.
        if (PARSER_compare(&at_ctx.parser, AT_COMMAND_LIST[idx].mode, AT_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
            // Execute callback and exit.
            AT_COMMAND_LIST[idx].callback();
            decode_success = 1;
            break;
        }
    }
    if (decode_success == 0) {
        _AT_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
        goto errors;
    }
errors:
    _AT_reset_parser();
    return;
}
#endif

/*** AT functions ***/

#ifdef ATM
/*******************************************************************/
void AT_init(void) {
    // Local variables.
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
    // Init context.
    _AT_reset_parser();
    // Init USART.
    usart_config.baud_rate = AT_USART_BAUD_RATE;
    usart_config.nvic_priority = NVIC_PRIORITY_AT;
    usart_config.rxne_callback = &_AT_fill_rx_buffer;
    usart_status = USART_init(AT_USART_INSTANCE, &GPIO_AT_USART, &usart_config);
    USART_stack_error(ERROR_BASE_USART_AT);
    // Start reception.
    usart_status = USART_enable_rx(AT_USART_INSTANCE);
    USART_stack_error(ERROR_BASE_USART_AT);
}
#endif

#ifdef ATM
/*******************************************************************/
void AT_task(void) {
    // Local variables.
    USART_status_t usart_status = USART_SUCCESS;
    // Trigger decoding function if line end found.
    if (at_ctx.line_end_flag != 0) {
        // Stop reception.
        usart_status = USART_disable_rx(AT_USART_INSTANCE);
        USART_stack_error(ERROR_BASE_USART_AT);
        // Decode and execute command.
        _AT_decode();
        // Start reception.
        usart_status = USART_enable_rx(AT_USART_INSTANCE);
        USART_stack_error(ERROR_BASE_USART_AT);
    }
}
#endif

#ifdef ATM
/*******************************************************************/
void AT_print_dl_payload(sfx_u8* dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
    // Local variables.
    uint8_t idx = 0;
    // Print DL payload.
    _AT_reply_add_string("+RX=");
    for (idx = 0; idx < dl_payload_size; idx++) {
        _AT_reply_add_value(dl_payload[idx], STRING_FORMAT_HEXADECIMAL, 0);
    }
    _AT_reply_add_string(" (RSSI=");
    _AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
    _AT_reply_add_string("dBm)");
    _AT_reply_send();
}
#endif
