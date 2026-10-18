#include <assert.h>
#include <stddef.h>

#include "pmsis.h"
#include "spi_driver.h"

static int open_result;
static int open_calls;
static int pad_calls;

void pi_pad_function_set(int pad, int function) {
    assert(pad >= PI_PAD_056 && pad <= PI_PAD_059);
    assert(function == PI_PAD_FUNC2);
    ++pad_calls;
}

void pi_spi_conf_init(struct pi_spi_conf *conf) {
    *conf = (struct pi_spi_conf){0};
}

void pi_open_from_conf(pi_device_t *device, struct pi_spi_conf *conf) {
    device->config = conf;
}

int pi_spi_open(pi_device_t *device) {
    const struct pi_spi_conf *conf = device->config;
    assert(conf != NULL);
    assert(conf->itf == 2);
    assert(conf->cs == 0);
    assert(conf->is_slave == PI_SPI_IS_SLAVE);
    ++open_calls;
    return open_result;
}

void pi_spi_send(pi_device_t *device, void *data, size_t bits, int flags) {
    (void)device;
    (void)data;
    (void)bits;
    (void)flags;
}

void pi_spi_receive(pi_device_t *device, void *data, size_t bits, int flags) {
    (void)device;
    (void)data;
    (void)bits;
    (void)flags;
}

int main(void) {
    open_result = -1;
    assert(spi_slave_driver_init() == -1);
    assert(open_calls == 1);
    assert(pad_calls == 4);

    open_result = 0;
    assert(spi_slave_driver_init() == 0);
    assert(open_calls == 2);
    assert(pad_calls == 8);
    return 0;
}
