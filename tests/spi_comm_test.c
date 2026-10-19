#include "FreeRTOS.h"
#include "deck.h"
#include "spi_comm.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
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
static uint8_t captured_frames[8][97];
static size_t captured_frame_count;

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
    (void)timeout_ticks;
    exchange_calls++;
    if (length == sizeof(captured_frames[0]) && tx != NULL &&
        captured_frame_count < sizeof(captured_frames) / sizeof(captured_frames[0])) {
        memcpy(captured_frames[captured_frame_count], tx, length);
        captured_frame_count++;
    }
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

static uint8_t hex_nibble(char value) {
    if (value >= '0' && value <= '9')
        return (uint8_t)(value - '0');
    if (value >= 'a' && value <= 'f')
        return (uint8_t)(value - 'a' + 10);
    if (value >= 'A' && value <= 'F')
        return (uint8_t)(value - 'A' + 10);
    assert(false);
    return 0;
}

static void assert_frame_matches_hex_file(const uint8_t *frame,
                                          const char *path) {
    char line[2 * 97 + 2];
    FILE *file = fopen(path, "r");
    assert(file != NULL);
    assert(fgets(line, sizeof(line), file) != NULL);
    assert(fclose(file) == 0);
    assert(strlen(line) == 2 * 97 || strlen(line) == 2 * 97 + 1);
    for (size_t index = 0; index < 97; index++) {
        uint8_t expected = (uint8_t)((hex_nibble(line[index * 2]) << 4) |
                                     hex_nibble(line[index * 2 + 1]));
        assert(frame[index] == expected);
    }
}

static void build_gazebo_pose_scan_frames(void) {
    aug_pose_t pose = {0};
    pose.id = 0;
    pose.timestamp = 66100;
    pose.pos[0] = -0.5f;
    pose.pos[1] = -0.5f;
    pose.pos[2] = -3.09641285e-17f;
    for (size_t direction = 0; direction < 4; direction++)
        for (size_t row = 0; row < 8; row++)
            pose.tof[direction][row] = -1;
    pose.tof[0][3] = 1900;
    pose.tof[1][3] = 900;
    pose.tof[2][3] = 1350;
    pose.tof[3][3] = 900;

    gpio_mode = 0;
    gpio_reads = 0;
    fake_ticks = 0;
    exchange_calls = 0;
    exchange_fail_at = 0;
    begin_calls = 0;
    begin_fail_at = 0;
    captured_frame_count = 0;
    assert(spi_send_aug_pose(&pose));
    assert(spi_send_scan_info(0));
    assert(captured_frame_count == 2);
}

static void test_actual_stm32_builders_match_gazebo_golden(
    const char *pose_path, const char *scan_path) {
    build_gazebo_pose_scan_frames();
    assert_frame_matches_hex_file(captured_frames[0], pose_path);
    assert_frame_matches_hex_file(captured_frames[1], scan_path);
}

static void print_hex_frame(const uint8_t *frame) {
    for (size_t index = 0; index < 97; index++)
        printf("%02x", frame[index]);
    putchar('\n');
}

int main(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "--dump-gazebo-golden") == 0) {
        build_gazebo_pose_scan_frames();
        print_hex_frame(captured_frames[0]);
        print_hex_frame(captured_frames[1]);
        return 0;
    }
    if (argc == 3) {
        test_actual_stm32_builders_match_gazebo_golden(argv[1], argv[2]);
        return 0;
    }
    assert(argc == 1);
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
