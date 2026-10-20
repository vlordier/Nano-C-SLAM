/* UDP implementation of the public UWB API for Linux/CrazySim SITL. */

#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "task.h"
#include "uwb_api.h"
#include "uwb_sitl_protocol.h"

#define SITL_MAX_DATAGRAM                                                      \
    (UWB_SITL_HEADER_SIZE + UWB_SITL_MAX_PAYLOAD + UWB_SITL_CRC_SIZE)
#define SITL_QUEUE_CAPACITY 8U
#define SITL_REGISTER_TIMEOUT_MS 3000U
#define SITL_REQUEST_TIMEOUT_MS 1200U
#define SITL_MAX_SYNC_AGE_MS 1500U
#define SITL_MAX_FUTURE_SKEW_NS 50000000ULL
#define SITL_MAX_SIM_AGE_NS 1000000000ULL

typedef struct {
    uwb_sitl_packet_t packet;
    uint8_t occupied;
} packet_slot_t;

TaskHandle_t uwbTaskHandle = NULL;
static pthread_mutex_t state_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t state_changed = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t tx_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t receiver_thread;
static int socket_fd = -1;
static struct sockaddr_in broker_address;
static uint8_t own_id;
static uint8_t initialized;
static uint8_t registered;
static uint16_t configured_local_port;
static uint8_t responder_enabled;
static uint8_t acknowledge_enabled = 1U;
static uint32_t next_sequence = 1U;
static uint64_t sim_time_ns;
static struct timespec last_sync_host;
static packet_slot_t message_queue;
static packet_slot_t measurement_queue;
static packet_slot_t response_queue[SITL_QUEUE_CAPACITY];
static uwb_sitl_freshness_guard_t freshness[10][6][6];

static uint32_t next_packet_sequence(void) {
    uint32_t result;
    pthread_mutex_lock(&state_lock);
    result = next_sequence++;
    if (next_sequence == 0U)
        next_sequence = 1U;
    pthread_mutex_unlock(&state_lock);
    return result;
}

static int parse_port(const char *name, unsigned long fallback,
                      uint16_t *port) {
    const char *value = getenv(name);
    char *end = NULL;
    unsigned long parsed = fallback;
    if (value != NULL && value[0] != '\0') {
        errno = 0;
        parsed = strtoul(value, &end, 10);
        if (errno != 0 || end == value || *end != '\0')
            return -1;
    }
    if (parsed == 0U || parsed > 65535U)
        return -1;
    *port = (uint16_t)parsed;
    return 0;
}

static int send_packet(const uwb_sitl_packet_t *packet) {
    uint8_t bytes[SITL_MAX_DATAGRAM];
    size_t length = 0U;
    if (uwb_sitl_encode(packet, bytes, sizeof(bytes), &length) != 0)
        return -1;
    pthread_mutex_lock(&tx_lock);
    const ssize_t sent = sendto(socket_fd, bytes, length, 0,
                                (const struct sockaddr *)&broker_address,
                                sizeof(broker_address));
    pthread_mutex_unlock(&tx_lock);
    return sent == (ssize_t)length ? 0 : -1;
}

static uwb_sitl_packet_t make_packet(uwb_sitl_kind_t kind, uint8_t destination,
                                     const uint8_t *payload, uint16_t length,
                                     uint32_t sequence) {
    uwb_sitl_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.kind = kind;
    packet.sequence = sequence;
    packet.sim_time_ns = sim_time_ns;
    packet.source_id = own_id;
    packet.destination_id = destination;
    packet.payload_length = length;
    if (length != 0U)
        memcpy(packet.payload, payload, length);
    return packet;
}

static void push_response(const uwb_sitl_packet_t *packet) {
    for (size_t i = 0U; i < SITL_QUEUE_CAPACITY; ++i) {
        if (response_queue[i].occupied == 0U) {
            response_queue[i].packet = *packet;
            response_queue[i].occupied = 1U;
            return;
        }
    }
    response_queue[packet->sequence % SITL_QUEUE_CAPACITY].packet = *packet;
    response_queue[packet->sequence % SITL_QUEUE_CAPACITY].occupied = 1U;
}

static int is_fresh(const uwb_sitl_packet_t *packet) {
    const uint8_t kind = (uint8_t)packet->kind;
    if (kind >= 10U || packet->source_id > 5U || packet->destination_id > 5U)
        return 0;
    return uwb_sitl_accept_fresh(
               &freshness[kind][packet->source_id][packet->destination_id],
               packet, sim_time_ns, SITL_MAX_SIM_AGE_NS) == 0;
}

static void make_ack_message(const uwb_sitl_packet_t *request,
                             uwb_sitl_packet_t *response) {
    const uint8_t code = request->payload[3];
    const uint8_t data_length = (uint8_t)(request->payload_length - 4U);
    uint8_t ack[UWB_SITL_MAX_PAYLOAD] = {
        0xDEU, own_id, request->source_id,
        code == UWB_ACK_MSG ? UWB_ACK_RES_MSG : UWB_ACK2_RES_MSG, 0U};
    uint8_t length = 4U;
    if (code == UWB_ACK_MSG) {
        if (data_length > 0U)
            ack[4] = request->payload[4];
        length = 5U;
    } else if (code == UWB_ACK2_MSG) {
        ack[4] = data_length;
        length = 5U;
    }
    *response = make_packet(UWB_SITL_MESSAGE_RESPONSE, request->source_id, ack,
                            length, request->sequence);
    response->sim_time_ns = sim_time_ns;
}

static void *receiver_main(void *unused) {
    (void)unused;
    uint8_t bytes[SITL_MAX_DATAGRAM];
    for (;;) {
        struct sockaddr_in sender;
        socklen_t sender_length = (socklen_t)sizeof(sender);
        const ssize_t received =
            recvfrom(socket_fd, bytes, sizeof(bytes), 0,
                     (struct sockaddr *)&sender, &sender_length);
        if (received < 0) {
            if (errno == EINTR)
                continue;
            if (socket_fd < 0)
                break;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                uint8_t port_bytes[2] = {(uint8_t)(configured_local_port >> 8),
                                         (uint8_t)configured_local_port};
                uwb_sitl_packet_t registration = make_packet(
                    UWB_SITL_REGISTER, UWB_SITL_BROKER_ID, port_bytes,
                    sizeof(port_bytes), next_packet_sequence());
                registration.sim_time_ns = 0U;
                (void)send_packet(&registration);
            }
            continue;
        }
        if (sender.sin_family != AF_INET ||
            sender.sin_addr.s_addr != broker_address.sin_addr.s_addr ||
            sender.sin_port != broker_address.sin_port)
            continue;
        uwb_sitl_packet_t packet;
        if (uwb_sitl_decode(bytes, (size_t)received, &packet) != 0)
            continue;
        pthread_mutex_lock(&state_lock);
        if (packet.kind == UWB_SITL_REGISTER_ACK &&
            packet.source_id == UWB_SITL_BROKER_ID &&
            packet.destination_id == own_id && packet.sim_time_ns != 0U) {
            registered = 1U;
            sim_time_ns = packet.sim_time_ns;
            (void)clock_gettime(CLOCK_MONOTONIC, &last_sync_host);
            pthread_cond_broadcast(&state_changed);
            pthread_mutex_unlock(&state_lock);
            continue;
        }
        if (packet.kind == UWB_SITL_TIME_SYNC &&
            packet.destination_id == own_id) {
            sim_time_ns = packet.sim_time_ns;
            (void)clock_gettime(CLOCK_MONOTONIC, &last_sync_host);
            pthread_cond_broadcast(&state_changed);
            pthread_mutex_unlock(&state_lock);
            continue;
        }
        if (packet.sim_time_ns > sim_time_ns &&
            packet.sim_time_ns - sim_time_ns <= SITL_MAX_FUTURE_SKEW_NS) {
            sim_time_ns = packet.sim_time_ns;
        }
        if (packet.destination_id != own_id || !is_fresh(&packet)) {
            pthread_mutex_unlock(&state_lock);
            continue;
        }
        if (packet.kind == UWB_SITL_MESSAGE_DELIVERY) {
            message_queue.packet = packet;
            message_queue.occupied = 1U;
            if (acknowledge_enabled != 0U && responder_enabled != 0U &&
                packet.payload_length >= 4U &&
                (packet.payload[3] == UWB_ACK_MSG ||
                 packet.payload[3] == UWB_ACK2_MSG)) {
                uwb_sitl_packet_t ack;
                make_ack_message(&packet, &ack);
                pthread_mutex_unlock(&state_lock);
                (void)send_packet(&ack);
                pthread_mutex_lock(&state_lock);
            }
        } else if (packet.kind == UWB_SITL_MEASUREMENT) {
            measurement_queue.packet = packet;
            measurement_queue.occupied = 1U;
        } else if (packet.kind == UWB_SITL_RANGE_RESPONSE ||
                   packet.kind == UWB_SITL_MESSAGE_RESPONSE ||
                   packet.kind == UWB_SITL_ERROR) {
            push_response(&packet);
        }
        pthread_cond_broadcast(&state_changed);
        pthread_mutex_unlock(&state_lock);
    }
    return NULL;
}

static void deadline_after_ms(struct timespec *deadline, uint32_t timeout_ms) {
    (void)clock_gettime(CLOCK_REALTIME, deadline);
    deadline->tv_sec += (time_t)(timeout_ms / 1000U);
    deadline->tv_nsec += (long)(timeout_ms % 1000U) * 1000000L;
    if (deadline->tv_nsec >= 1000000000L) {
        ++deadline->tv_sec;
        deadline->tv_nsec -= 1000000000L;
    }
}

static uwb_err_code_e wait_response(uint8_t kind, uint8_t peer,
                                    uint32_t sequence, uint32_t timeout_ms,
                                    uwb_sitl_packet_t *result) {
    struct timespec deadline;
    deadline_after_ms(&deadline, timeout_ms);
    pthread_mutex_lock(&state_lock);
    for (;;) {
        for (size_t i = 0U; i < SITL_QUEUE_CAPACITY; ++i) {
            packet_slot_t *slot = &response_queue[i];
            if (slot->occupied != 0U &&
                (slot->packet.kind == UWB_SITL_ERROR ||
                 slot->packet.sequence == sequence) &&
                (slot->packet.kind == UWB_SITL_ERROR ||
                 slot->packet.source_id == peer) &&
                slot->packet.destination_id == own_id) {
                *result = slot->packet;
                slot->occupied = 0U;
                pthread_mutex_unlock(&state_lock);
                if (result->kind == UWB_SITL_ERROR)
                    return UWB_RX_TIMEOUT;
                return result->kind == (uwb_sitl_kind_t)kind ? UWB_SUCCESS
                                                             : UWB_INVALID_RSP;
            }
        }
        struct timespec now;
        (void)clock_gettime(CLOCK_MONOTONIC, &now);
        const int64_t sync_age_ms =
            ((int64_t)now.tv_sec - (int64_t)last_sync_host.tv_sec) * 1000 +
            ((int64_t)now.tv_nsec - (int64_t)last_sync_host.tv_nsec) / 1000000;
        if (sync_age_ms > SITL_MAX_SYNC_AGE_MS) {
            pthread_mutex_unlock(&state_lock);
            return UWB_RX_TIMEOUT;
        }
        const int waited =
            pthread_cond_timedwait(&state_changed, &state_lock, &deadline);
        if (waited == ETIMEDOUT) {
            pthread_mutex_unlock(&state_lock);
            return UWB_RX_TIMEOUT;
        }
    }
}

static uwb_err_code_e send_uwb_message(UWB_message message,
                                       uint32_t *sequence) {
    if (message.ctrl != 0xDEU || message.src != own_id || message.dest > 5U ||
        message.dest == own_id || message.data_len > MSG_DATA_LEN)
        return UWB_TX_ERROR;
    uint8_t payload[UWB_SITL_MAX_PAYLOAD];
    const size_t length = 4U + message.data_len;
    payload[0] = message.ctrl;
    payload[1] = message.src;
    payload[2] = message.dest;
    payload[3] = message.code;
    memcpy(payload + 4U, message.data, message.data_len);
    pthread_mutex_lock(&state_lock);
    const int time_valid = sim_time_ns != 0U;
    pthread_mutex_unlock(&state_lock);
    if (!time_valid)
        return UWB_TX_ERROR;
    const uint32_t packet_sequence = next_packet_sequence();
    uwb_sitl_packet_t packet =
        make_packet(UWB_SITL_MESSAGE_REQUEST, message.dest, payload,
                    (uint16_t)length, packet_sequence);
    if (send_packet(&packet) != 0)
        return UWB_TX_ERROR;
    if (sequence != NULL)
        *sequence = packet_sequence;
    return UWB_SUCCESS;
}

uwb_err_code_e uwb_api_init(uint8_t node_id) {
    if (node_id > 5U)
        return UWB_ERROR;
    pthread_mutex_lock(&state_lock);
    if (initialized != 0U) {
        pthread_mutex_unlock(&state_lock);
        return own_id == node_id ? UWB_SUCCESS : UWB_ERROR;
    }
    own_id = node_id;
    pthread_mutex_unlock(&state_lock);

    const char *host = getenv("NCS_SITL_UWB_BROKER_HOST");
    if (host == NULL || host[0] == '\0')
        host = "192.168.5.1";
    uint16_t broker_port, local_port;
    if (parse_port("NCS_SITL_UWB_BROKER_PORT", 19952U, &broker_port) != 0 ||
        parse_port("NCS_SITL_UWB_LOCAL_PORT", 19953U + node_id, &local_port) !=
            0)
        return UWB_ERROR;
    configured_local_port = local_port;
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
        return UWB_ERROR;
    struct sockaddr_in local_address;
    memset(&local_address, 0, sizeof(local_address));
    local_address.sin_family = AF_INET;
    local_address.sin_addr.s_addr = htonl(INADDR_ANY);
    local_address.sin_port = htons(local_port);
    memset(&broker_address, 0, sizeof(broker_address));
    broker_address.sin_family = AF_INET;
    broker_address.sin_port = htons(broker_port);
    if (inet_pton(AF_INET, host, &broker_address.sin_addr) != 1 ||
        bind(socket_fd, (const struct sockaddr *)&local_address,
             sizeof(local_address)) != 0) {
        close(socket_fd);
        socket_fd = -1;
        return UWB_ERROR;
    }
    struct timeval receive_timeout = {.tv_sec = 0, .tv_usec = 100000};
    (void)setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
                     sizeof(receive_timeout));
    if (pthread_create(&receiver_thread, NULL, receiver_main, NULL) != 0) {
        close(socket_fd);
        socket_fd = -1;
        return UWB_ERROR;
    }
    uint8_t port_bytes[2] = {(uint8_t)(local_port >> 8), (uint8_t)local_port};
    uwb_sitl_packet_t registration =
        make_packet(UWB_SITL_REGISTER, UWB_SITL_BROKER_ID, port_bytes,
                    sizeof(port_bytes), next_packet_sequence());
    if (send_packet(&registration) != 0)
        return UWB_TX_ERROR;
    struct timespec deadline;
    deadline_after_ms(&deadline, SITL_REGISTER_TIMEOUT_MS);
    pthread_mutex_lock(&state_lock);
    while (registered == 0U || sim_time_ns == 0U) {
        if (pthread_cond_timedwait(&state_changed, &state_lock, &deadline) ==
            ETIMEDOUT) {
            pthread_mutex_unlock(&state_lock);
            DEBUG_PRINT(
                "NCS_SITL_DEVICE_ERROR device=uwb reason=no_sim_clock\n");
            return UWB_RX_TIMEOUT;
        }
    }
    pthread_mutex_unlock(&state_lock);
    pthread_mutex_lock(&state_lock);
    initialized = 1U;
    pthread_mutex_unlock(&state_lock);
    DEBUG_PRINT("NCS_SITL_DEVICE_READY device=uwb node=%u port=%u\n",
                (unsigned int)node_id, (unsigned int)local_port);
    return UWB_SUCCESS;
}

uwb_err_code_e uwb_send_msg(UWB_message message, uint32_t tx_delay,
                            uint8_t response_expected) {
    (void)tx_delay;
    (void)response_expected;
    return send_uwb_message(message, NULL);
}

uwb_err_code_e uwb_send_msg_wait_rsp(UWB_message message, uint32_t tx_delay,
                                     uint8_t *rx_buffer,
                                     uint8_t *rx_buffer_length) {
    (void)tx_delay;
    if (rx_buffer == NULL || rx_buffer_length == NULL)
        return UWB_RX_ERROR;
    memset(rx_buffer, 0, RX_BUF_LEN);
    *rx_buffer_length = 0U;
    uint32_t sequence = 0U;
    uwb_err_code_e status = send_uwb_message(message, &sequence);
    if (status != UWB_SUCCESS)
        return status;
    uwb_sitl_packet_t response;
    status = wait_response(UWB_SITL_MESSAGE_RESPONSE, message.dest, sequence,
                           SITL_REQUEST_TIMEOUT_MS, &response);
    if (status != UWB_SUCCESS)
        return status;
    if (response.payload_length < 4U ||
        response.payload_length > RX_BUF_LEN - 2U)
        return UWB_INVALID_RSP;
    memcpy(rx_buffer, response.payload, response.payload_length);
    *rx_buffer_length = (uint8_t)(response.payload_length + 2U);
    return UWB_SUCCESS;
}

static uwb_err_code_e do_ranging(uint8_t target_id, uint32_t *range_dst_mm) {
    if (target_id > 5U || target_id == own_id)
        return UWB_TX_ERROR;
    pthread_mutex_lock(&state_lock);
    if (sim_time_ns == 0U) {
        pthread_mutex_unlock(&state_lock);
        return UWB_RX_TIMEOUT;
    }
    pthread_mutex_unlock(&state_lock);
    const uint32_t sequence = next_packet_sequence();
    uwb_sitl_packet_t request =
        make_packet(UWB_SITL_RANGE_REQUEST, target_id, NULL, 0U, sequence);
    if (send_packet(&request) != 0)
        return UWB_TX_ERROR;
    uwb_sitl_packet_t response;
    uwb_err_code_e status =
        wait_response(UWB_SITL_RANGE_RESPONSE, target_id, sequence,
                      SITL_REQUEST_TIMEOUT_MS, &response);
    if (status != UWB_SUCCESS)
        return status;
    const uint32_t range_mm = ((uint32_t)response.payload[0] << 24) |
                              ((uint32_t)response.payload[1] << 16) |
                              ((uint32_t)response.payload[2] << 8) |
                              response.payload[3];
    if (range_mm == 0U)
        return UWB_INVALID_RSP;
    if (range_dst_mm != NULL)
        *range_dst_mm = range_mm;
    return UWB_SUCCESS;
}

uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id,
                                             uwb_node_coordinates_t node_pos) {
    (void)node_pos;
    return do_ranging(target_id, NULL);
}

uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id,
                                             uwb_node_coordinates_t node_pos,
                                             uint32_t *range_dst_mm) {
    (void)node_pos;
    if (range_dst_mm == NULL)
        return UWB_ERROR;
    *range_dst_mm = 0U;
    return do_ranging(target_id, range_dst_mm);
}

uwb_err_code_e get_msg_from_queue(UWB_message *message, TickType_t timeout) {
    if (message == NULL)
        return UWB_RX_ERROR;
    struct timespec deadline;
    deadline_after_ms(&deadline, (uint32_t)timeout);
    pthread_mutex_lock(&state_lock);
    while (message_queue.occupied == 0U) {
        if (pthread_cond_timedwait(&state_changed, &state_lock, &deadline) ==
            ETIMEDOUT) {
            pthread_mutex_unlock(&state_lock);
            return UWB_RX_TIMEOUT;
        }
    }
    const uwb_sitl_packet_t packet = message_queue.packet;
    message_queue.occupied = 0U;
    pthread_mutex_unlock(&state_lock);
    memset(message, 0, sizeof(*message));
    message->ctrl = packet.payload[0];
    message->src = packet.payload[1];
    message->dest = packet.payload[2];
    message->code = packet.payload[3];
    message->data_len = (uint8_t)(packet.payload_length - 4U);
    memcpy(message->data, packet.payload + 4U, message->data_len);
    return UWB_SUCCESS;
}

uwb_err_code_e get_msrm_from_queue(UWB_measurement *measurement,
                                   TickType_t timeout) {
    if (measurement == NULL)
        return UWB_RX_ERROR;
    struct timespec deadline;
    deadline_after_ms(&deadline, (uint32_t)timeout);
    pthread_mutex_lock(&state_lock);
    while (measurement_queue.occupied == 0U) {
        if (pthread_cond_timedwait(&state_changed, &state_lock, &deadline) ==
            ETIMEDOUT) {
            pthread_mutex_unlock(&state_lock);
            return UWB_RX_TIMEOUT;
        }
    }
    const uwb_sitl_packet_t packet = measurement_queue.packet;
    measurement_queue.occupied = 0U;
    pthread_mutex_unlock(&state_lock);
    const uint32_t range = ((uint32_t)packet.payload[0] << 24) |
                           ((uint32_t)packet.payload[1] << 16) |
                           ((uint32_t)packet.payload[2] << 8) |
                           packet.payload[3];
    const int32_t x =
        (int32_t)(((uint32_t)packet.payload[4] << 24) |
                  ((uint32_t)packet.payload[5] << 16) |
                  ((uint32_t)packet.payload[6] << 8) | packet.payload[7]);
    const int32_t y =
        (int32_t)(((uint32_t)packet.payload[8] << 24) |
                  ((uint32_t)packet.payload[9] << 16) |
                  ((uint32_t)packet.payload[10] << 8) | packet.payload[11]);
    const int32_t z =
        (int32_t)(((uint32_t)packet.payload[12] << 24) |
                  ((uint32_t)packet.payload[13] << 16) |
                  ((uint32_t)packet.payload[14] << 8) | packet.payload[15]);
    measurement->src_id = packet.source_id;
    /* The existing Nano call site divides these float fields by 1000. */
    measurement->posx = (float)x;
    measurement->posy = (float)y;
    measurement->posz = (float)z;
    measurement->range = (float)range / 1000.0f;
    return UWB_SUCCESS;
}

uwb_err_code_e uwb_responder_on(void) {
    pthread_mutex_lock(&state_lock);
    responder_enabled = 1U;
    pthread_mutex_unlock(&state_lock);
    return UWB_SUCCESS;
}
uwb_err_code_e uwb_responder_off(void) {
    pthread_mutex_lock(&state_lock);
    responder_enabled = 0U;
    pthread_mutex_unlock(&state_lock);
    return UWB_SUCCESS;
}
uwb_err_code_e reset_msg_queue(void) {
    pthread_mutex_lock(&state_lock);
    message_queue.occupied = 0U;
    pthread_mutex_unlock(&state_lock);
    return UWB_SUCCESS;
}
uwb_err_code_e uwb_set_gpio0(uint8_t value) {
    (void)value;
    return UWB_SUCCESS;
}
uwb_err_code_e uwb_set_gpio1(uint8_t value) {
    (void)value;
    return UWB_SUCCESS;
}
uwb_err_code_e uwb_enable_acknowlege(void) {
    pthread_mutex_lock(&state_lock);
    acknowledge_enabled = 1U;
    pthread_mutex_unlock(&state_lock);
    return UWB_SUCCESS;
}
uwb_err_code_e uwb_disable_acknowlege(void) {
    pthread_mutex_lock(&state_lock);
    acknowledge_enabled = 0U;
    pthread_mutex_unlock(&state_lock);
    return UWB_SUCCESS;
}
