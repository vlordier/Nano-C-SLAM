#ifndef TEST_FLIGHT_COMMANDER_H
#define TEST_FLIGHT_COMMANDER_H

#include <stdbool.h>

typedef enum { modeDisable, modeAbs, modeVelocity } setpoint_mode_type_t;

typedef struct {
    setpoint_mode_type_t x;
    setpoint_mode_type_t y;
    setpoint_mode_type_t z;
    setpoint_mode_type_t yaw;
} setpoint_mode_t;

typedef struct {
    float x;
    float y;
    float z;
} setpoint_vector_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} setpoint_attitude_rate_t;

typedef struct {
    setpoint_mode_t mode;
    setpoint_vector_t position;
    setpoint_vector_t velocity;
    setpoint_attitude_rate_t attitudeRate;
    bool velocity_body;
} setpoint_t;

#define COMMANDER_PRIORITY_EXTRX 3

void commanderSetSetpoint(setpoint_t *setpoint, int priority);
void commanderRelaxPriority(void);

#endif
