#ifndef TOF_FRESHNESS_H
#define TOF_FRESHNESS_H

#include "FreeRTOS.h"

#include <stdbool.h>
#include <stdint.h>

#define TOF_FRESHNESS_SENSOR_COUNT 4u

typedef struct {
    TickType_t last_success[TOF_FRESHNESS_SENSOR_COUNT];
    uint8_t seen_mask;
} tof_freshness_t;

void tof_freshness_reset(volatile tof_freshness_t *state);
bool tof_freshness_mark_success(volatile tof_freshness_t *state,
                                uint8_t sensor_id, TickType_t now);
bool tof_freshness_all_fresh(const volatile tof_freshness_t *state,
                             TickType_t now, TickType_t max_age);

#endif
