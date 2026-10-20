/* Fail-closed seam for the future host/GVSOC SPI bridge. */
#include "spi_comm_sitl.h"

#include "debug.h"

bool ncs_sitl_spi_exchange(size_t length, const uint8_t *tx, uint8_t *rx) {
    (void)length;
    (void)tx;
    (void)rx;
    DEBUG_PRINT(
        "NCS_SITL_DEVICE_MISSING device=gap9_spi provider=unconfigured\n");
    return false;
}

int ncs_sitl_gap9_busy(void) { return -1; }
