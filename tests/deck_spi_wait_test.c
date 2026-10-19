#include "deck_spi_wait.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

typedef struct fake_wait fake_wait_t;

typedef struct {
    fake_wait_t *owner;
    uint8_t index;
} fake_event_t;

struct fake_wait {
    uint32_t ticks;
    uint32_t duration[2];
    uint32_t timeout_seen[2];
    uint8_t calls[2];
    fake_event_t events[2];
};

static uint32_t fake_ticks(void *context) {
    return ((fake_wait_t *)context)->ticks;
}

static bool fake_wait_event(void *context, uint32_t event_timeout) {
    fake_event_t *event = context;
    fake_wait_t *fake = event->owner;
    uint8_t index = event->index;
    fake->timeout_seen[index] = event_timeout;
    fake->calls[index]++;
    if (fake->duration[index] > event_timeout) {
        fake->ticks += event_timeout;
        return false;
    }
    fake->ticks += fake->duration[index];
    return true;
}

static deck_spi_wait_ops_t operations(fake_wait_t *fake) {
    fake->events[0] = (fake_event_t){.owner = fake, .index = 0};
    fake->events[1] = (fake_event_t){.owner = fake, .index = 1};
    return (deck_spi_wait_ops_t){
        .context = fake,
        .tx_event = &fake->events[0],
        .rx_event = &fake->events[1],
        .get_ticks = fake_ticks,
        .wait_event = fake_wait_event,
    };
}

static void test_both_dma_events_share_one_deadline(void) {
    fake_wait_t fake = {.duration = {4, 6}};
    deck_spi_wait_ops_t ops = operations(&fake);
    assert(deck_spi_wait_for_tx_rx(&ops, 10));
    assert(fake.timeout_seen[0] == 10);
    assert(fake.timeout_seen[1] == 6);
    assert(fake.ticks == 10);
}

static void test_rx_cannot_extend_deadline_after_tx(void) {
    fake_wait_t fake = {.ticks = UINT32_MAX - 2U, .duration = {4, 7}};
    deck_spi_wait_ops_t ops = operations(&fake);
    assert(!deck_spi_wait_for_tx_rx(&ops, 10));
    assert(fake.timeout_seen[0] == 10);
    assert(fake.timeout_seen[1] == 6);
    assert(fake.ticks == 7U);
}

static void test_tx_timeout_skips_rx_wait(void) {
    fake_wait_t fake = {.duration = {11, 0}};
    deck_spi_wait_ops_t ops = operations(&fake);
    assert(!deck_spi_wait_for_tx_rx(&ops, 10));
    assert(fake.calls[0] == 1);
    assert(fake.calls[1] == 0);
}

static void test_invalid_operations_fail_closed(void) {
    fake_wait_t fake = {0};
    deck_spi_wait_ops_t ops = operations(&fake);
    assert(!deck_spi_wait_for_tx_rx(NULL, 10));
    ops.wait_event = NULL;
    assert(!deck_spi_wait_for_tx_rx(&ops, 10));
}

int main(void) {
    test_both_dma_events_share_one_deadline();
    test_rx_cannot_extend_deadline_after_tx();
    test_tx_timeout_skips_rx_wait();
    test_invalid_operations_fail_closed();
    return 0;
}
