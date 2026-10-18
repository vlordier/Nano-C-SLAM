#ifndef TEST_GAP9_SPI_PMSIS_H
#define TEST_GAP9_SPI_PMSIS_H

#include <stddef.h>

typedef struct {
    void *config;
    void *data;
} pi_device_t;

struct pi_spi_conf {
    int wordsize;
    int big_endian;
    int max_baudrate;
    int polarity;
    int phase;
    int itf;
    int cs;
    int dummy_clk_cycle;
    int dummy_clk_cycle_mode;
    int is_slave;
};

typedef enum { PI_SPI_WORDSIZE_8 = 8 } pi_spi_wordsize_e;
typedef enum { PI_SPI_POLARITY_0 = 0 } pi_spi_polarity_e;
typedef enum { PI_SPI_PHASE_0 = 0 } pi_spi_phase_e;

#define PI_SPI_DUMMY_CLK_CYCLE_BEFORE_CS 0
#define PI_SPI_DUMMY_CLK_CYCLE_MODE 0
#define PI_SPI_IS_SLAVE 1
#define PI_PAD_056 56
#define PI_PAD_057 57
#define PI_PAD_058 58
#define PI_PAD_059 59
#define PI_PAD_FUNC2 2
#define PI_SPI_CS_NONE 0

void pi_pad_function_set(int pad, int function);
void pi_spi_conf_init(struct pi_spi_conf *conf);
void pi_open_from_conf(pi_device_t *device, struct pi_spi_conf *conf);
int pi_spi_open(pi_device_t *device);
void pi_spi_send(pi_device_t *device, void *data, size_t bits, int flags);
void pi_spi_receive(pi_device_t *device, void *data, size_t bits, int flags);

#endif
