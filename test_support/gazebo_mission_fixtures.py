"""Reusable mission-status fixtures for Gazebo bundle validation tests."""

from __future__ import annotations

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    MissionPhase,
    MissionStatusEvent,
)


def successful_mapping_mission_events(
    run_id: str, drone_name: str
) -> list[MissionStatusEvent]:
    """Return a valid bounded-path mission with measured stable landing."""
    return [
        _event(run_id, drone_name, 0, MissionPhase.WAITING, 0.0),
        _event(run_id, drone_name, 1, MissionPhase.TAKEOFF, 0.1, altitude_m=0.0),
        _event(run_id, drone_name, 2, MissionPhase.HOVER, 0.2, altitude_m=0.5),
        _event(
            run_id,
            drone_name,
            3,
            MissionPhase.LAND,
            0.8,
            altitude_m=0.05,
            mapping_path_complete=True,
            odometry_stamp_s=0.8,
            landing_stable_since_s=0.8,
        ),
        _event(
            run_id,
            drone_name,
            4,
            MissionPhase.LAND,
            1.0,
            altitude_m=0.05,
            mapping_path_complete=True,
            odometry_stamp_s=1.0,
            landing_stable_since_s=0.8,
        ),
        _event(
            run_id,
            drone_name,
            5,
            MissionPhase.LAND,
            1.2,
            altitude_m=0.05,
            mapping_path_complete=True,
            odometry_stamp_s=1.2,
            landing_stable_since_s=0.8,
        ),
        _event(
            run_id,
            drone_name,
            6,
            MissionPhase.COMPLETE,
            1.4,
            altitude_m=0.05,
            mapping_path_complete=True,
            odometry_stamp_s=1.4,
            landing_stable_since_s=0.8,
        ),
    ]


def _event(
    run_id: str,
    drone_name: str,
    sequence: int,
    phase: MissionPhase,
    sim_time_s: float,
    *,
    altitude_m: float | None = None,
    mapping_path_complete: bool = False,
    odometry_stamp_s: float | None = None,
    landing_stable_since_s: float | None = None,
) -> MissionStatusEvent:
    return MissionStatusEvent(
        run_id=run_id,
        drone_name=drone_name,
        sequence=sequence,
        sim_time_s=sim_time_s,
        phase=phase,
        altitude_m=altitude_m,
        mapping_path_requested=True,
        mapping_path_complete=mapping_path_complete,
        landing_height_m=0.1,
        odometry_stamp_s=odometry_stamp_s,
        landing_stable_since_s=landing_stable_since_s,
    )
