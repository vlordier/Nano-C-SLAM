#ifndef TEST_FLIGHT_CRTP_HIGH_LEVEL_H
#define TEST_FLIGHT_CRTP_HIGH_LEVEL_H

#include <stdbool.h>

int crtpCommanderHighLevelTakeoffWithVelocity(float height_m, float velocity_m_s,
                                              bool relative);
int crtpCommanderHighLevelLand(float height_m, float duration_s);

#endif
