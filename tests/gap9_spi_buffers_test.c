#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "spi_buffers.h"
#include "spi_protocol.h"

#define RX_SIZE SPIBUF_SIZE
#define TX_SIZE (3 * SPIBUF_SIZE + 1)

static int allocation_call;
static int fail_on_call;
static int released_count;
static int released_size;
static uint8_t allocations[2][TX_SIZE];

static void *test_allocate(int32_t size) {
    assert(size == RX_SIZE || size == TX_SIZE);
    ++allocation_call;
    if (allocation_call == fail_on_call)
        return NULL;
    return allocations[allocation_call - 1];
}

static void test_release(void *buffer, int32_t size) {
    assert(buffer == allocations[0]);
    released_size = size;
    ++released_count;
}

static void reset(int fail_at) {
    allocation_call = 0;
    fail_on_call = fail_at;
    released_count = 0;
    released_size = 0;
}

int main(void) {
    uint8_t *receive_buffer = (uint8_t *)1;
    uint8_t *transmit_buffer = (uint8_t *)1;

    reset(1);
    assert(spi_buffers_allocate(&receive_buffer, &transmit_buffer, test_allocate,
                                test_release) == -1);
    assert(receive_buffer == NULL && transmit_buffer == NULL);
    assert(allocation_call == 1 && released_count == 0);

    reset(2);
    assert(spi_buffers_allocate(&receive_buffer, &transmit_buffer, test_allocate,
                                test_release) == -1);
    assert(receive_buffer == NULL && transmit_buffer == NULL);
    assert(allocation_call == 2 && released_count == 1);
    assert(released_size == RX_SIZE);

    reset(0);
    assert(spi_buffers_allocate(&receive_buffer, &transmit_buffer, test_allocate,
                                test_release) == 0);
    assert(receive_buffer == allocations[0]);
    assert(transmit_buffer == allocations[1]);
    assert(allocation_call == 2 && released_count == 0);

    assert(spi_buffers_allocate(NULL, &transmit_buffer, test_allocate, test_release) ==
           -1);
    assert(spi_buffers_allocate(&receive_buffer, &transmit_buffer, NULL,
                                test_release) == -1);
    return 0;
}
