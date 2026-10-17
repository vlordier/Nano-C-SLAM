/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <stdbool.h>
#include <stdint.h>

#include "../icp/icp.h"
#include "../slam/graph-based-slam.h"
#include "pmsis.h"

// Augmented pose structure
typedef struct {
    int16_t id;
    int32_t timestamp;
    float pos[3];
    int16_t tof[4][8];
} pcl_pose_t;

// Structure that stores the optimization information
typedef struct {
    int16_t number_of_poses;
    int16_t main_graph_size;
    int16_t max_subgraph_size;
    int16_t time_ms;
    int16_t has_nan;
    int16_t main_solver_returned;
    int16_t successful_solver_call_count;
    int16_t completed_solver_iteration_count;
    int16_t optimization_completed;
} slam_job_t;

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

/**
 * @brief Add a new augmented pose to the pose table.
 *
 * @param pose The new augmented pose to be added containing the timestamp,
 * drone's (x,y,yaw) and the ToF line information from 4 sensors
 * @return True when the pose has a unique nonnegative ID, finite coordinates,
 * valid range sentinels, and is stored.
 */
bool pcl_add_augmented_pose(pcl_pose_t pose);

/**
 * @brief Transform a scan from pose table entries to a point cloud.
 *
 * @param start_id The id where the scan starts in the pose table
 * @param end_id The id where the scan ends in the pose table
 * @param points Pointer to the point cloud where the scan should be stored
 * @param range_limit Only project distance measurements below this threshold
 */
void pcl_extract_scan(int16_t start_id, int16_t end_id, icp_points_t *points,
                      int16_t range_limit);

/**
 * @brief Add a loop closure constraint if its undirected endpoint pair is new.
 *
 * The first accepted measurement for a pair wins; same-direction and reversed
 * duplicates, self-edges, invalid indices, and invalid measurements are
 * rejected without changing storage.
 *
 * @return true when a unique constraint was stored, otherwise false.
 */
bool pcl_add_constraint(int16_t from_node, int16_t to_node, float *icp);

/** Add a finite external constraint if its undirected endpoint pair is new. */
bool pcl_add_external_constraint(constraint c);

/**
 * @brief Start hierarchical pose graph optimization.
 *
 * @param job_info Pointer to where to store information about the graph
 * optimization
 */
void pcl_optimize(slam_job_t *job_info);

/**
 * @brief Move only the pose information (i.e., x, y, yaw) from the pose table
 * to an array.
 *
 * @param x The extracted poses will be stored here, and each pose occupies 3
 * entries in the array
 * @param from_id Start extracting from this id
 * @param nr_of_poses How many poses to extract starting from the given id
 * @return True when the requested pose range exists and was copied.
 */
bool pcl_to_poses(float *x, int16_t from_id, int16_t nr_of_poses);

/**
 * @brief Get the number of poses in the pose table.
 *
 * @return Returns the number of poses currently in the graph / pose table
 */
int16_t pcl_get_pose_count();

/**
 * @brief Tries to match the scan associated to the pose with ID "cur_pose_id"
 * with one of the previously acquired internal scans and derive an intra-drone
 * loop closure constraint.
 *
 * @param cur_pose_id The ID of the scan pose to be matched
 */
void check_internal_constraints(uint16_t cur_pose_id);

/**
 * @brief Map an incoming STM32 pose ID to an internal pose-table index and add
 * that index to the scan history.
 *
 * @param pose_id The ID of the scan pose
 */
void add_scan_pose_id(uint16_t pose_id);

/**
 * @brief Get the internal pose-table index for the idx-th stored scan pose.
 *
 * @param idx The index of the scan pose id in the buffer
 */
int16_t pcl_get_scan_pose_id(uint16_t idx);

/**
 * @brief Returns the scan pose count.
 *
 * @return Number of stored scan poses
 */
uint16_t pcl_get_scan_pose_count();

/** Return the number of valid pose-graph constraints currently stored. */
int32_t pcl_get_constraint_count(void);

/** Return stored endpoint indices for a constraint, or false for invalid args.
 */
bool pcl_get_constraint_endpoints(int32_t index, int16_t *from, int16_t *to);

/** Return constraints in the most recent graph submitted to ls_slam. */
int32_t pcl_get_optimizer_input_constraint_count(void);

/** Return unique undirected endpoint pairs in the most recent ls_slam input. */
int32_t pcl_get_optimizer_input_unique_edge_count(void);

/**
 * Return compact graph-node endpoints from the most recent ls_slam input.
 * The indices refer to the reduced graph passed to ls_slam, not pose-table IDs.
 */
bool pcl_get_optimizer_input_endpoints(int32_t index, int16_t *from,
                                       int16_t *to);

/**
 * @brief Prints the stored augmented poses (only for debugging).
 */
void print_poses();

#endif
