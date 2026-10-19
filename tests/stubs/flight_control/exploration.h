#ifndef TEST_FLIGHT_EXPLORATION_H
#define TEST_FLIGHT_EXPLORATION_H

#include <stdint.h>

void explore(uint16_t tof_data[64], float lines[3], float sides[2], float distance_back,
             float fly_commands[3], float dtd_distance);

#endif
