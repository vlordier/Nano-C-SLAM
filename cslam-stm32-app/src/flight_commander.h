/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef FLIGHT_CMD_H
#define FLIGHT_CMD_H

typedef enum {
    FLIGHT_STATUS_WAITING_FOR_ENABLE = 0,
    FLIGHT_STATUS_TAKEOFF = 1,
    FLIGHT_STATUS_ACTIVE = 2,
    FLIGHT_STATUS_LANDING = 3,
    FLIGHT_STATUS_COMPLETE = 4,
    FLIGHT_STATUS_TAKEOFF_FAILED = 5,
    FLIGHT_STATUS_LAND_FAILED = 6,
    FLIGHT_STATUS_ABORTED_BEFORE_TAKEOFF = 7,
} flight_status_t;

extern volatile flight_status_t flight_status;

void flight_task(void *parameters);

/**
 * @brief Enable flight and start mission.
 */
void enable_commander();

#endif
