/*
 * Explicit fail-closed UWB provider for the Linux SITL build.
 *
 * The real UWB transport must be backed by timestamped simulated peer
 * exchanges before tasks can proceed. This provider is intentionally unable
 * to report a successful initialization or range measurement.
 */

#include <string.h>

#include "debug.h"
#include "uwb_api.h"

uwb_err_code_e uwb_api_init(uint8_t node_id) {
    DEBUG_PRINT(
        "NCS_SITL_DEVICE_MISSING device=uwb provider=unconfigured node=%u\n",
        (unsigned int)node_id);
    return UWB_ERROR;
}

uwb_err_code_e uwb_send_msg(UWB_message message, uint32_t tx_delay,
                            uint8_t response_expected) {
    (void)message;
    (void)tx_delay;
    (void)response_expected;
    return UWB_TX_ERROR;
}

uwb_err_code_e uwb_send_msg_wait_rsp(UWB_message message, uint32_t tx_delay,
                                     uint8_t *rx_buffer,
                                     uint8_t *rx_buffer_length) {
    (void)message;
    (void)tx_delay;
    if (rx_buffer != NULL) {
        memset(rx_buffer, 0, RX_BUF_LEN);
    }
    if (rx_buffer_length != NULL) {
        *rx_buffer_length = 0;
    }
    return UWB_RX_TIMEOUT;
}

uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id,
                                             uwb_node_coordinates_t node_pos) {
    (void)target_id;
    (void)node_pos;
    return UWB_RX_TIMEOUT;
}

uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id,
                                             uwb_node_coordinates_t node_pos,
                                             uint32_t *range_dst_mm) {
    (void)target_id;
    (void)node_pos;
    if (range_dst_mm != NULL) {
        *range_dst_mm = 0;
    }
    return UWB_RX_TIMEOUT;
}

uwb_err_code_e get_msg_from_queue(UWB_message *message, TickType_t timeout) {
    (void)timeout;
    if (message != NULL) {
        memset(message, 0, sizeof(*message));
    }
    return UWB_RX_TIMEOUT;
}

uwb_err_code_e get_msrm_from_queue(UWB_measurement *measurement,
                                   TickType_t timeout) {
    (void)timeout;
    if (measurement != NULL) {
        memset(measurement, 0, sizeof(*measurement));
    }
    return UWB_RX_TIMEOUT;
}

uwb_err_code_e uwb_responder_on(void) { return UWB_ERROR; }

uwb_err_code_e uwb_responder_off(void) { return UWB_ERROR; }

uwb_err_code_e reset_msg_queue(void) { return UWB_ERROR; }

uwb_err_code_e uwb_set_gpio0(uint8_t value) {
    (void)value;
    return UWB_ERROR;
}

uwb_err_code_e uwb_set_gpio1(uint8_t value) {
    (void)value;
    return UWB_ERROR;
}

uwb_err_code_e uwb_enable_acknowlege(void) { return UWB_ERROR; }

uwb_err_code_e uwb_disable_acknowlege(void) { return UWB_ERROR; }
