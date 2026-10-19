#include "bounded_batch.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct send_log {
    uint8_t attempted[8];
    size_t count;
    uint8_t fail_value;
} send_log_t;

static bool record_send(const void *item, void *context) {
    const uint8_t *value = item;
    send_log_t *log = context;
    log->attempted[log->count++] = *value;
    return *value != log->fail_value;
}

static void test_budget_preserves_ordered_backlog(void) {
    uint8_t items[] = {10, 20, 30};
    uint16_t count = 3;
    send_log_t log = {0};

    assert(
        bounded_batch_flush(items, sizeof(items[0]), &count, 3, 1, record_send, &log));
    assert(log.count == 1 && log.attempted[0] == 10);
    assert(count == 2 && items[0] == 20 && items[1] == 30);

    assert(
        bounded_batch_flush(items, sizeof(items[0]), &count, 3, 1, record_send, &log));
    assert(log.count == 2 && log.attempted[1] == 20);
    assert(count == 1 && items[0] == 30);
}

static void test_failed_item_and_tail_remain_queued(void) {
    uint8_t items[] = {10, 20, 30};
    uint16_t count = 3;
    send_log_t log = {.fail_value = 20};

    assert(
        !bounded_batch_flush(items, sizeof(items[0]), &count, 3, 3, record_send, &log));
    assert(log.count == 2 && log.attempted[0] == 10 && log.attempted[1] == 20);
    assert(count == 2 && items[0] == 20 && items[1] == 30);
}

static void test_invalid_buffer_state_fails_closed(void) {
    uint8_t items[] = {10};
    uint16_t count = 2;
    send_log_t log = {0};

    assert(
        !bounded_batch_flush(items, sizeof(items[0]), &count, 1, 1, record_send, &log));
    assert(count == 2 && log.count == 0);
}

int main(void) {
    test_budget_preserves_ordered_backlog();
    test_failed_item_and_tail_remain_queued();
    test_invalid_buffer_state_fails_closed();
    return 0;
}
