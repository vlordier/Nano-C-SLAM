#ifndef TEST_FLIGHT_TOF_DAQ_H
#define TEST_FLIGHT_TOF_DAQ_H

#include <stdint.h>

extern int16_t tof_data_buf[4][8][8];
float get_min_dtd_dist(void);

#endif
