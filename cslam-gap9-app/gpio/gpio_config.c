/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license. See the LICENSE file for details.
 */

#include "pmsis.h"

#define GPIO_CF_BUSY PI_PAD_063

void gpio_init(void) {
    pi_pad_function_set(GPIO_CF_BUSY, PI_PAD_FUNC1);
    pi_gpio_pin_configure(GPIO_CF_BUSY, PI_GPIO_OUTPUT);
    pi_gpio_pin_write(GPIO_CF_BUSY, 0);
}

void set_cf_gpio(int8_t value) { pi_gpio_pin_write(GPIO_CF_BUSY, value != 0); }
