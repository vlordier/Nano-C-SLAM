#include "FreeRTOS.h"
#include "deck.h"
#include "spi_comm.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint32_t fake_ticks;
static int gpio_mode;
static uint32_t gpio_reads;
static uint32_t pose_response_exchanges;
static uint32_t other_exchanges;
static uint32_t exchange_calls;
static uint32_t exchange_fail_at;
static uint32_t begin_calls;
static uint32_t begin_fail_at;

void test_debug_print(const char *format, ...) {
    (void)format;
}

int digitalRead(uint32_t pin) {
    assert(pin == DECK_GPIO_IO2);
    gpio_reads++;
    if (gpio_mode == 0)
        return 0;
    if (gpio_mode == 1)
        return gpio_reads == 1 ? 1 : 0;
    if (gpio_mode == 2)
        return 1;
    return 0;
}

void digitalWrite(uint32_t pin, int value) {
    (void)pin;
    (void)value;
}

void spiBeginTransaction(uint32_t baudrate) {
    (void)baudrate;
}
void spiEndTransaction(void) {}

bool spiBeginTransactionTimeout(uint32_t baudrate, uint32_t timeout_ticks) {
    (void)baudrate;
    (void)timeout_ticks;
    begin_calls++;
    return begin_calls != begin_fail_at;
}

bool spiExchangeTimeout(size_t length, const uint8_t *tx, uint8_t *rx,
                        uint32_t timeout_ticks) {
    (void)tx;
    (void)timeout_ticks;
    exchange_calls++;
    if (length != 1 && length != 97 && length != 3 * sizeof(float))
        other_exchanges++;
    if (length == 3 * sizeof(float)) {
        const float expected[3] = {1.25f, -2.5f, 3.75f};
        memcpy(rx, expected, sizeof(expected));
        pose_response_exchanges++;
    } else if (rx != NULL) {
        memset(rx, 0, length);
    }
    return exchange_calls != exchange_fail_at;
}

TickType_t xTaskGetTickCount(void) {
    return fake_ticks;
}
void vTaskDelay(TickType_t ticks) {
    fake_ticks += ticks;
}
void vTaskDelayUntil(TickType_t *previous_wake_time, TickType_t increment) {
    fake_ticks += increment;
    *previous_wake_time = fake_ticks;
}

static void test_timeout_preserves_pose_output(void) {
    float poses[3] = {91.0f, 92.0f, 93.0f};
    gpio_mode = 0;
    gpio_reads = 0;
    fake_ticks = 0;
    pose_response_exchanges = 0;
    exchange_calls = 0;
    exchange_fail_at = 0;
    begin_calls = 0;
    begin_fail_at = 0;

    assert(!spi_rcv_poses(poses, 7, 1));
    assert(poses[0] == 91.0f && poses[1] == 92.0f && poses[2] == 93.0f);
    assert(pose_response_exchanges == 0);
}

static void test_ready_pose_response_is_copied(void) {
    float poses[3] = {0};
    gpio_mode = 1;
    gpio_reads = 0;
    fake_ticks = 0;
    pose_response_exchanges = 0;
    exchange_calls = 0;
    exchange_fail_at = 0;
    begin_calls = 0;
    begin_fail_at = 0;

    assert(spi_rcv_poses(poses, 7, 1));
    assert(poses[0] == 1.25f && poses[1] == -2.5f && poses[2] == 3.75f);
    assert(pose_response_exchanges == 1);
}

static void test_pose_response_length_is_bounded(void) {
    float poses[3] = {4.0f, 5.0f, 6.0f};
    gpio_mode = 1;
    gpio_reads = 0;
    pose_response_exchanges = 0;
    exchange_calls = 0;
    exchange_fail_at = 0;
    begin_calls = 0;
    begin_fail_at = 0;
    assert(!spi_rcv_poses(poses, 0, 51));
    assert(poses[0] == 4.0f && poses[1] == 5.0f && poses[2] == 6.0f);
    assert(pose_response_exchanges == 0);
}

static void test_scan_busy_timeout_returns_failure(void) {
    gpio_mode = 2;
    gpio_reads = 0;
    fake_ticks = 0;
    assert(!spi_send_scan_info(3));
}

static void test_spi_dma_failure_stops_scan_command(void) {
    gpio_mode = 0;
    gpio_reads = 0;
    fake_ticks = 0;
    exchange_calls = 0;
    exchange_fail_at = 2;
    begin_calls = 0;
    begin_fail_at = 0;
    assert(!spi_send_scan_info(3));
    assert(gpio_reads == 0);
}

static void test_spi_mutex_timeout_preserves_pose_output(void) {
    float poses[3] = {11.0f, 12.0f, 13.0f};
    gpio_mode = 1;
    gpio_reads = 0;
    begin_calls = 0;
    begin_fail_at = 1;
    exchange_calls = 0;
    exchange_fail_at = 0;
    pose_response_exchanges = 0;
    assert(!spi_rcv_poses(poses, 7, 1));
    assert(poses[0] == 11.0f && poses[1] == 12.0f && poses[2] == 13.0f);
    assert(exchange_calls == 0 && pose_response_exchanges == 0);
}

static void test_optimizer_busy_timeout_does_not_clock_response(void) {
    slam_job_t job;
    memset(&job, 0xA5, sizeof(job));
    gpio_mode = 2;
    gpio_reads = 0;
    fake_ticks = 0;
    other_exchanges = 0;
    exchange_calls = 0;
    exchange_fail_at = 0;
    begin_calls = 0;
    begin_fail_at = 0;
    assert(!spi_send_opt_cmd(&job));
    assert(other_exchanges == 0);
    const uint8_t *bytes = (const uint8_t *)&job;
    for (size_t i = 0; i < sizeof(job); i++)
        assert(bytes[i] == 0);
}

static void test_optimizer_response_dma_failure_does_not_copy_partial_job(void) {
    slam_job_t job;
    memset(&job, 0xA5, sizeof(job));
    gpio_mode = 0;
    gpio_reads = 0;
    fake_ticks = 0;
    exchange_calls = 0;
    exchange_fail_at = 4; /* dummy, command, dummy, response */
    begin_calls = 0;
    begin_fail_at = 0;
    assert(!spi_send_opt_cmd(&job));
    const uint8_t *bytes = (const uint8_t *)&job;
    for (size_t i = 0; i < sizeof(job); i++)
        assert(bytes[i] == 0);
}

int main(void) {
    test_timeout_preserves_pose_output();
    test_ready_pose_response_is_copied();
    test_pose_response_length_is_bounded();
    test_scan_busy_timeout_returns_failure();
    test_spi_dma_failure_stops_scan_command();
    test_spi_mutex_timeout_preserves_pose_output();
    test_optimizer_busy_timeout_does_not_clock_response();
    test_optimizer_response_dma_failure_does_not_copy_partial_job();
    return 0;
}
