#ifndef SPI_WAIT_H
#define SPI_WAIT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    void *context;
    int (*read_level)(void *context);
    uint32_t (*get_ticks)(void *context);
    void (*delay_ticks)(void *context, uint32_t ticks);
} spi_wait_ops_t;

/** Wait for a GPIO level with a wrap-safe tick deadline. */
bool spi_wait_for_level(const spi_wait_ops_t *ops, int expected_level,
                        uint32_t timeout_ticks, uint32_t poll_ticks);

#endif
