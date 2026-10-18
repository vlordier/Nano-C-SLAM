#include "spi_buffers.h"

#include "spi_protocol.h"

#define SPI_TX_BUFFER_SIZE (3 * SPIBUF_SIZE + 1)

int spi_buffers_allocate(uint8_t **receive_buffer, uint8_t **transmit_buffer,
                         spi_buffer_alloc_fn allocate,
                         spi_buffer_free_fn release) {
    if (receive_buffer == NULL || transmit_buffer == NULL || allocate == NULL ||
        release == NULL)
        return -1;

    *receive_buffer = NULL;
    *transmit_buffer = NULL;

    *receive_buffer = allocate(SPIBUF_SIZE);
    if (*receive_buffer == NULL) return -1;

    *transmit_buffer = allocate(SPI_TX_BUFFER_SIZE);
    if (*transmit_buffer != NULL) return 0;

    release(*receive_buffer, SPIBUF_SIZE);
    *receive_buffer = NULL;
    return -1;
}
