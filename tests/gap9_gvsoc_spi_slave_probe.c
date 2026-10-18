#include <stdint.h>
#include <stdio.h>

#include "pmsis.h"
#include "spi_driver.h"
#include "spi_protocol.h"

static uint8_t received_frame[SPI_PKT_SIZE];

static int probe(void) {
    if (spi_slave_driver_init() != 0) {
        printf("NANO_C_SLAM_SPI_SLAVE_PROBE:error stage=open\n");
        pmsis_exit(-1);
        return -1;
    }

    printf("NANO_C_SLAM_SPI_SLAVE_PROBE:armed frame=%d\n", SPI_PKT_SIZE);
    spi_rcv_blocking(received_frame, sizeof(received_frame));
    printf("NANO_C_SLAM_SPI_SLAVE_PROBE:frame=received bytes=%d first=%02x cmd=%02x "
           "id=%02x%02x checksum=%02x%02x%02x%02x last=%02x\n",
           SPI_PKT_SIZE, received_frame[0], received_frame[1], received_frame[2],
           received_frame[3], received_frame[4], received_frame[5], received_frame[6],
           received_frame[7], received_frame[SPI_PKT_SIZE - 1]);

    const int valid = received_frame[0] == 'S' && received_frame[1] == 1 &&
                      received_frame[2] == 0 && received_frame[3] == 0 &&
                      received_frame[4] == 0 && received_frame[5] == 0 &&
                      received_frame[6] == 0 && received_frame[7] == 0 &&
                      received_frame[SPI_PKT_SIZE - 1] == 'E';
    printf("NANO_C_SLAM_SPI_SLAVE_PROBE:payload_%s\n", valid ? "ok" : "mismatch");
    pmsis_exit(valid ? 0 : -1);
    return valid ? 0 : -1;
}

int main(void) {
    return probe();
}
