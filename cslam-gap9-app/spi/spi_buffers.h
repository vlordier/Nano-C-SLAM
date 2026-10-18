#ifndef NANO_C_SLAM_SPI_BUFFERS_H
#define NANO_C_SLAM_SPI_BUFFERS_H

#include <stddef.h>
#include <stdint.h>

typedef void *(*spi_buffer_alloc_fn)(int32_t size);
typedef void (*spi_buffer_free_fn)(void *buffer, int32_t size);

/* Allocate the fixed receive and transmit buffers used by the SPI protocol. */
int spi_buffers_allocate(uint8_t **receive_buffer, uint8_t **transmit_buffer,
                         spi_buffer_alloc_fn allocate,
                         spi_buffer_free_fn release);

#endif
