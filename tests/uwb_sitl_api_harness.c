#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include "uwb_api.h"

#ifdef NCS_SITL_TESTING
int ncs_sitl_test_receiver_signals_blocked(void);
#endif

static void pause_ms(long milliseconds) {
    const struct timespec delay = {.tv_sec = milliseconds / 1000L,
                                   .tv_nsec = (milliseconds % 1000L) * 1000000L};
    (void)nanosleep(&delay, NULL);
}

int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
#ifdef NCS_SITL_TESTING
    if (strcmp(argv[1], "signal-mask") == 0) {
        sigset_t caller_mask;
        sigemptyset(&caller_mask);
        if (pthread_sigmask(SIG_SETMASK, &caller_mask, NULL) != 0 ||
            uwb_api_init(0U) != UWB_SUCCESS) {
            fputs("signal-mask probe failed to initialize\n", stderr);
            return 12;
        }
        for (unsigned attempt = 0U; attempt < 100U; ++attempt) {
            if (ncs_sitl_test_receiver_signals_blocked()) {
                puts("UWB receiver blocks FreeRTOS scheduler signals");
                return 0;
            }
            pause_ms(1L);
        }
        fputs("UWB receiver inherited the caller signal mask\n", stderr);
        return 13;
    }
#endif
    if (strcmp(argv[1], "missing") == 0) {
        if (uwb_api_init(0U) != UWB_SUCCESS)
            return 3;
        UWB_message message = {
            .ctrl = 0xDEU, .src = 0U, .dest = 2U, .code = UWB_ACK2_MSG, .data_len = 1U};
        uint8_t response[RX_BUF_LEN];
        uint8_t response_length = 0U;
        if (uwb_send_msg_wait_rsp(message, 0U, response, &response_length) !=
                UWB_RX_TIMEOUT ||
            response_length != 0U) {
            fputs("missing peer did not fail closed\n", stderr);
            return 9;
        }
        puts("missing peer returned UWB_RX_TIMEOUT");
        return 0;
    }
    if (strcmp(argv[1], "range-failure") == 0 || strcmp(argv[1], "stale-range") == 0) {
        if (uwb_api_init(0U) != UWB_SUCCESS)
            return 3;
        if (strcmp(argv[1], "stale-range") == 0)
            pause_ms(350L);
        uint32_t range_mm = 123U;
        if (uwb_do_4way_ranging_with_node(1U, (uwb_node_coordinates_t){0}, &range_mm) !=
                UWB_RX_TIMEOUT ||
            range_mm != 0U) {
            fputs("invalid range did not fail closed\n", stderr);
            return 10;
        }
        puts("invalid range returned UWB_RX_TIMEOUT with zero range");
        return 0;
    }
    if (strcmp(argv[1], "idle-peer") == 0) {
        if (uwb_api_init(1U) != UWB_SUCCESS)
            return 3;
        pause_ms(1500L);
        return 0;
    }
    const int wrong_destination = strcmp(argv[1], "wrong-dest") == 0;
    const uint8_t node_id =
        wrong_destination ? 0U : (uint8_t)strtoul(argv[1], NULL, 10);
    if (uwb_api_init(node_id) != UWB_SUCCESS) {
        fputs("init failed\n", stderr);
        return 3;
    }
    if (node_id == 0U) {
        pause_ms(250L);
        UWB_message message = {.ctrl = 0xDEU,
                               .src = 0U,
                               .dest = 1U,
                               .code = UWB_ACK2_MSG,
                               .data = {0x11U, 0x22U, 0x33U},
                               .data_len = 3U};
        uint8_t response[RX_BUF_LEN];
        uint8_t response_length = 0U;
        if (uwb_send_msg_wait_rsp(message, 0U, response, &response_length) !=
                UWB_SUCCESS ||
            response_length != 7U || response[3] != UWB_ACK2_RES_MSG ||
            response[4] != 3U) {
            fputs("ACK2 exchange failed\n", stderr);
            return 4;
        }
        uint32_t range_mm = 0U;
        const uwb_err_code_e range_status =
            uwb_do_4way_ranging_with_node(1U, (uwb_node_coordinates_t){0}, &range_mm);
        if (wrong_destination) {
            if (range_status != UWB_RX_TIMEOUT || range_mm != 0U) {
                fputs("wrong-destination range did not fail closed\n", stderr);
                return 11;
            }
            puts("wrong-destination range returned timeout and zero");
            return 0;
        }
        if (range_status != UWB_SUCCESS || range_mm != 1000U) {
            fputs("range exchange failed\n", stderr);
            return 5;
        }
        puts("node 0: ACK2 and 1000 mm range passed");
        return 0;
    }
    if (node_id != 1U || uwb_responder_on() != UWB_SUCCESS)
        return 6;
    UWB_message received;
    if (get_msg_from_queue(&received, 5000U) != UWB_SUCCESS || received.src != 0U ||
        received.dest != 1U || received.code != UWB_ACK2_MSG ||
        received.data_len != 3U || memcmp(received.data, "\x11\x22\x33", 3U) != 0) {
        fputs("message delivery failed\n", stderr);
        return 7;
    }
    UWB_measurement measurement;
    if (get_msrm_from_queue(&measurement, 5000U) != UWB_SUCCESS ||
        measurement.src_id != 0U || measurement.range != 1.0f ||
        measurement.posx != 2000.0f || measurement.posy != 500.0f) {
        fputs("measurement delivery failed\n", stderr);
        return 8;
    }
    puts("node 1: message and ranging measurement passed");
    return 0;
}
