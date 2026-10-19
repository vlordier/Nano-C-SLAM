#ifndef CONF_PARAMS_H
#define CONF_PARAMS_H

// Number of drones in the swarm
#define NR_OF_NODES 2

// UWB Re-transmission attempts
#define RETRY_TIME_MS 5

/* Per-token GAP9 work quotas bound token-owner time without dropping backlog. */
#define GAP9_AUG_POSES_PER_TOKEN 1
#define GAP9_EXTERNAL_AUG_POSES_PER_TOKEN 1
#define GAP9_POSE_UPDATES_PER_TOKEN 1

// Flight heigth
#define EXP_HEIGHT 0.7f

#endif
