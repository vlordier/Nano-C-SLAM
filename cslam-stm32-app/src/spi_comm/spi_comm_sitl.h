#ifndef NCS_SPI_COMM_SITL_H
#define NCS_SPI_COMM_SITL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Socket-backed transport to the GAP9/GVSOC bridge. Returns false until a
 * bridge is connected and has accepted the exact SPI transaction. */
bool ncs_sitl_spi_exchange(size_t length, const uint8_t *tx, uint8_t *rx);
int ncs_sitl_gap9_busy(void);

#endif
