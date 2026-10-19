#include "spi_wait.h"

bool spi_wait_for_level(const spi_wait_ops_t *ops, int expected_level,
                        uint32_t timeout_ticks, uint32_t poll_ticks) {
    if (ops == 0 || ops->read_level == 0 || ops->get_ticks == 0 ||
        ops->delay_ticks == 0 || timeout_ticks == 0 || poll_ticks == 0) {
        return false;
    }

    uint32_t start = ops->get_ticks(ops->context);
    while (ops->read_level(ops->context) != expected_level) {
        if ((uint32_t)(ops->get_ticks(ops->context) - start) >= timeout_ticks)
            return false;
        ops->delay_ticks(ops->context, poll_ticks);
    }
    return true;
}
