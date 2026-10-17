"""Tests for the hardware-free Gazebo flight sequence."""

import unittest
import uuid

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_path import (
    MappingPath,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
    MissionConfig,
    MissionPhase,
    MissionSequence,
    MissionStatusEvent,
    create_mapping_start_altitude_limit,
    create_mission_config,
    create_run_id,
    mission_exit_code,
    validate_drone_name,
    validate_drone_names,
    validate_mission_odometry,
    validate_mission_status_events,
)


class MissionSequenceTests(unittest.TestCase):
    def test_validates_drone_name_before_constructing_topics(self) -> None:
        self.assertEqual(validate_drone_name("cf3"), "cf3")
        self.assertEqual(
            validate_drone_names(list(SUPPORTED_DRONE_NAMES)), SUPPORTED_DRONE_NAMES
        )
        for invalid_name in ("cf", "cf2", "cf1/topic", "/cf1", "cf1;stop", 1):
            with (
                self.subTest(invalid_name=invalid_name),
                self.assertRaisesRegex(ValueError, "drone_name"),
            ):
                validate_drone_name(invalid_name)

    def test_rejects_invalid_or_duplicate_sensor_bridge_drones(self) -> None:
        for invalid_names in ([], ["cf1", "cf1"], ["cf1", "cf2"], "cf1"):
            with (
                self.subTest(invalid_names=invalid_names),
                self.assertRaises(ValueError),
            ):
                validate_drone_names(invalid_names)

    def test_validates_mission_odometry_before_readiness(self) -> None:
        validate_mission_odometry(
            (1.0, -2.0, 0.1), 10.1, now_s=10.0, previous_stamp_s=10.1
        )
        validate_mission_odometry(
            (0.0, 0.0, 0.0), 9.5, now_s=10.0, previous_stamp_s=None
        )
        invalid_cases = (
            ((0.0, 0.0, float("nan")), 10.0, 10.0, None, "position"),
            ((0.0, 0.0, 0.0), float("inf"), 10.0, None, "timestamp"),
            ((0.0, 0.0, 0.0), 10.2, 10.0, None, "ahead"),
            ((0.0, 0.0, 0.0), 9.4, 10.0, None, "stale"),
            ((0.0, 0.0, 0.0), 9.9, 10.0, 10.0, "backwards"),
        )
        for position, stamp_s, now_s, previous_stamp_s, reason in invalid_cases:
            with (
                self.subTest(reason=reason),
                self.assertRaisesRegex(ValueError, reason),
            ):
                validate_mission_odometry(
                    position,
                    stamp_s,
                    now_s=now_s,
                    previous_stamp_s=previous_stamp_s,
                )

    def test_builds_config_from_numeric_ros_parameters(self) -> None:
        self.assertEqual(
            create_mission_config(0.7, 2),
            MissionConfig(hover_height_m=0.7, hover_duration_s=2.0),
        )
        for invalid_values in (("0.5", 2.0), (0.5, True), (float("nan"), 2.0)):
            with (
                self.subTest(invalid_values=invalid_values),
                self.assertRaises(ValueError),
            ):
                create_mission_config(*invalid_values)

    def test_validates_mapping_start_altitude_limit(self) -> None:
        self.assertEqual(create_mapping_start_altitude_limit(0.15), 0.15)
        for invalid_value in (True, "0.15", float("nan"), float("inf"), -0.01):
            with (
                self.subTest(invalid_value=invalid_value),
                self.assertRaises(ValueError),
            ):
                create_mapping_start_altitude_limit(invalid_value)

    def test_takeoff_hover_and_land_follow_measured_altitude(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=2.0))

        self.assertEqual(mission.step(0.0, None), 0.0)
        self.assertIs(mission.phase, MissionPhase.WAITING)
        self.assertEqual(mission.step(0.1, 0.0, 0.1), 0.5)
        self.assertIs(mission.phase, MissionPhase.TAKEOFF)
        self.assertEqual(mission.step(1.0, 0.5, 1.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.9, 0.5, 2.9), 0.0)
        self.assertEqual(mission.step(3.0, 0.5, 3.0), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)
        self.assertEqual(mission.step(3.1, 0.1, 3.1), 0.0)
        self.assertIs(mission.phase, MissionPhase.COMPLETE)
        self.assertEqual(mission.step(3.2, 0.1, 3.2), 0.0)

    def test_config_rejects_unreachable_landing_height(self) -> None:
        with self.assertRaisesRegex(ValueError, "below hover_height_m"):
            MissionConfig(hover_height_m=0.5, landing_height_m=0.5)

    def test_non_finite_altitude_fails_safe_with_zero_command(self) -> None:
        mission = MissionSequence()
        self.assertEqual(mission.step(0.0, float("nan"), 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry altitude is not finite")

    def test_backward_simulation_time_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(2.0, 0.0, 2.0), 0.5)
        self.assertEqual(mission.step(1.9, 0.0, 1.9), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "simulation time moved backwards")

    def test_non_finite_simulation_time_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(float("nan"), 0.0, 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "simulation time is not finite")

    def test_stale_odometry_fails_safe(self) -> None:
        mission = MissionSequence(MissionConfig(odometry_timeout_s=0.2))

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.1, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.21, 0.0, 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry data is stale")

    def test_odometry_timestamp_regression_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(0.1, 0.0, 0.1), 0.5)
        self.assertEqual(mission.step(0.2, 0.0, 0.05), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry timestamp moved backwards")

    def test_active_mission_fails_if_altitude_disappears(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.1, None), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry altitude is unavailable")

    def test_takeoff_timeout_stops_with_zero_command(self) -> None:
        mission = MissionSequence(MissionConfig(phase_timeout_s=1.0))

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(1.0, 0.0, 1.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "takeoff phase timed out")

    def test_external_path_failure_is_terminal_and_zero_command(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.fail("mapping waypoint timed out"), 0.0)

        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "mapping waypoint timed out")
        self.assertEqual(mission.step(1.0, 0.0, 1.0), 0.0)

    def test_already_airborne_mission_moves_to_hover_and_landing(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=0.5))

        self.assertEqual(mission.step(2.0, 0.6, 2.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.5, 0.6, 2.5), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)

    def test_mapping_path_holds_hover_timer_until_route_finishes(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=2.0))
        mission.step(0.0, 0.0, 0.0)
        mission.step(0.1, 0.5, 0.1)

        self.assertEqual(mission.step(1.0, 0.5, 1.0, hold_hover=True), 0.0)
        self.assertEqual(mission.step(5.0, 0.5, 5.0, hold_hover=True), 0.0)

        self.assertEqual(mission.step(6.9, 0.5, 6.9), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(7.0, 0.5, 7.0), -0.5)


class MissionStatusEventTests(unittest.TestCase):
    def setUp(self) -> None:
        self.run_id = create_run_id()

    def event(
        self,
        sequence: int,
        phase: MissionPhase,
        *,
        sim_time_s: float | None = None,
        altitude_m: float | None = 0.05,
        path_requested: bool = True,
        path_complete: bool = True,
        failure_reason: str | None = None,
    ) -> MissionStatusEvent:
        return MissionStatusEvent(
            run_id=self.run_id,
            drone_name="cf1",
            sequence=sequence,
            sim_time_s=float(sequence) if sim_time_s is None else sim_time_s,
            phase=phase,
            altitude_m=altitude_m,
            mapping_path_requested=path_requested,
            mapping_path_complete=path_complete,
            landing_height_m=0.1,
            failure_reason=failure_reason,
        )

    def successful_events(
        self, *, path_requested: bool = True
    ) -> list[MissionStatusEvent]:
        return [
            self.event(
                0,
                MissionPhase.WAITING,
                altitude_m=None,
                path_requested=path_requested,
                path_complete=False,
            ),
            self.event(
                1,
                MissionPhase.TAKEOFF,
                altitude_m=0.0,
                path_requested=path_requested,
                path_complete=False,
            ),
            self.event(
                2,
                MissionPhase.HOVER,
                altitude_m=0.5,
                path_requested=path_requested,
                path_complete=path_requested,
            ),
            self.event(
                3,
                MissionPhase.LAND,
                altitude_m=0.5,
                path_requested=path_requested,
                path_complete=path_requested,
            ),
            self.event(
                4,
                MissionPhase.COMPLETE,
                altitude_m=0.05,
                path_requested=path_requested,
                path_complete=path_requested,
            ),
        ]

    def test_generated_run_id_is_canonical_uuid(self) -> None:
        self.assertEqual(str(uuid.UUID(self.run_id)), self.run_id)

    def test_process_exit_code_fails_closed_on_nonterminal_and_failed_states(
        self,
    ) -> None:
        self.assertEqual(mission_exit_code(MissionPhase.COMPLETE), 0)
        self.assertEqual(mission_exit_code(MissionPhase.FAILED), 1)
        self.assertEqual(mission_exit_code(MissionPhase.WAITING), 1)
        self.assertEqual(
            mission_exit_code(MissionPhase.COMPLETE, interrupted=True), 130
        )

    def test_json_round_trip_preserves_terminal_evidence(self) -> None:
        events = self.successful_events()

        decoded = [MissionStatusEvent.from_json(event.to_json()) for event in events]
        terminal = validate_mission_status_events(decoded, self.run_id, "cf1")

        self.assertIs(terminal.phase, MissionPhase.COMPLETE)
        self.assertEqual(terminal.altitude_m, 0.05)

    def test_rejects_bad_identity_numbers_and_schema(self) -> None:
        valid = self.event(0, MissionPhase.TAKEOFF, altitude_m=0.0)
        raw = valid.to_json()
        invalid_payloads = (
            "not json",
            raw.replace(self.run_id, ""),
            raw.replace(self.run_id, "not-a-uuid"),
            raw.replace('"drone_name": "cf1"', '"drone_name": "cf2"'),
            raw.replace('"schema_version": 1', '"schema_version": 2'),
            raw.replace('"sequence": 0', '"sequence": true'),
            raw.replace('"sim_time_s": 0.0', '"sim_time_s": NaN'),
            raw[:-1] + ', "unrecognized": true}',
            raw[:-1] + ', "sequence": 0}',
        )
        for payload in invalid_payloads:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                MissionStatusEvent.from_json(payload)

    def test_rejects_nonfinite_values_and_missing_failed_reason(self) -> None:
        with self.assertRaisesRegex(ValueError, "sim_time_s"):
            self.event(0, MissionPhase.TAKEOFF, sim_time_s=float("inf"))
        with self.assertRaisesRegex(ValueError, "sim_time_s"):
            self.event(0, MissionPhase.TAKEOFF, sim_time_s=10**1000)
        with self.assertRaisesRegex(ValueError, "altitude_m"):
            self.event(0, MissionPhase.TAKEOFF, altitude_m=float("nan"))
        with self.assertRaisesRegex(ValueError, "failure_reason"):
            self.event(0, MissionPhase.FAILED, failure_reason=None)

    def test_success_requires_bounded_landing_and_completed_requested_path(
        self,
    ) -> None:
        for altitude, path_complete in ((0.11, True), (0.1, False), (None, True)):
            with (
                self.subTest(altitude=altitude, path_complete=path_complete),
                self.assertRaises(ValueError),
            ):
                self.event(
                    0,
                    MissionPhase.COMPLETE,
                    altitude_m=altitude,
                    path_complete=path_complete,
                )
        boundary = self.event(0, MissionPhase.COMPLETE, altitude_m=0.1)
        self.assertEqual(boundary.altitude_m, boundary.landing_height_m)
        no_path_events = self.successful_events(path_requested=False)
        no_path = no_path_events[-1]
        self.assertFalse(no_path.mapping_path_requested)
        with self.assertRaisesRegex(ValueError, "without requesting"):
            validate_mission_status_events(
                no_path_events, self.run_id, "cf1", require_mapping_path=True
            )

    def test_rejects_path_completion_without_a_requested_path(self) -> None:
        with self.assertRaisesRegex(ValueError, "unrequested mapping path"):
            self.event(
                0,
                MissionPhase.TAKEOFF,
                altitude_m=0.0,
                path_requested=False,
                path_complete=True,
            )

    def test_stream_rejects_missing_terminal_and_duplicate_or_reordered_events(
        self,
    ) -> None:
        waiting = self.event(
            0,
            MissionPhase.WAITING,
            altitude_m=None,
            path_complete=False,
        )
        takeoff = self.event(
            1,
            MissionPhase.TAKEOFF,
            altitude_m=0.0,
            path_complete=False,
        )
        hover = self.event(2, MissionPhase.HOVER, altitude_m=0.5)
        skipped = self.event(2, MissionPhase.HOVER, altitude_m=0.5)
        for events in (
            [],
            [waiting],
            [waiting, waiting],
            [waiting, skipped],
            [waiting, hover, takeoff],
        ):
            with self.subTest(events=events), self.assertRaises(ValueError):
                validate_mission_status_events(events, self.run_id, "cf1")

    def test_stream_rejects_time_regression_identity_mismatch_and_events_after_terminal(
        self,
    ) -> None:
        waiting = self.event(
            0,
            MissionPhase.WAITING,
            sim_time_s=1.0,
            altitude_m=None,
            path_complete=False,
        )
        takeoff = self.event(
            1,
            MissionPhase.TAKEOFF,
            sim_time_s=1.0,
            altitude_m=0.0,
            path_complete=False,
        )
        hover = self.event(2, MissionPhase.HOVER, sim_time_s=0.5, altitude_m=0.5)
        with self.assertRaisesRegex(ValueError, "backwards"):
            validate_mission_status_events(
                [waiting, takeoff, hover], self.run_id, "cf1"
            )
        with self.assertRaisesRegex(ValueError, "drone_name"):
            validate_mission_status_events(
                [self.event(0, MissionPhase.COMPLETE)], self.run_id, "cf3"
            )
        with self.assertRaisesRegex(ValueError, "run_id"):
            validate_mission_status_events(
                [self.event(0, MissionPhase.COMPLETE)], create_run_id(), "cf1"
            )
        complete = self.successful_events()
        following = self.event(
            5,
            MissionPhase.FAILED,
            altitude_m=0.05,
            failure_reason="unexpected trailing event",
        )
        with self.assertRaisesRegex(ValueError, "final event"):
            validate_mission_status_events([*complete, following], self.run_id, "cf1")

    def test_failed_terminal_event_is_not_success(self) -> None:
        failed = self.event(
            2,
            MissionPhase.FAILED,
            altitude_m=0.4,
            path_complete=False,
            failure_reason="landing timed out",
        )
        with self.assertRaisesRegex(ValueError, "mission failed"):
            validate_mission_status_events(
                [
                    self.event(
                        0,
                        MissionPhase.WAITING,
                        altitude_m=None,
                        path_complete=False,
                    ),
                    self.event(
                        1,
                        MissionPhase.TAKEOFF,
                        altitude_m=0.0,
                        path_complete=False,
                    ),
                    failed,
                ],
                self.run_id,
                "cf1",
            )


class MappingPathTests(unittest.TestCase):
    def test_tracks_square_with_bounded_world_frame_commands(self) -> None:
        path = MappingPath(2.0, -1.0, 0.0)

        command = path.step(0.1, 2.0, -1.0)
        self.assertAlmostEqual(command[0], 0.2)
        self.assertAlmostEqual(command[1], 0.0)

        self.assertEqual(path.step(1.0, 2.25, -1.0), (0.0, 0.0))
        self.assertEqual(path.waypoint_index, 1)
        command = path.step(1.1, 2.25, -1.0)
        self.assertAlmostEqual(command[0], 0.0)
        self.assertAlmostEqual(command[1], 0.2)

    def test_completes_only_after_all_waypoints_are_reached(self) -> None:
        offsets = ((0.1, 0.0),)
        path = MappingPath(0.0, 0.0, 0.0, offsets_m=offsets)

        path.step(0.1, 0.0, 0.0)
        self.assertFalse(path.complete)
        self.assertEqual(path.step(0.2, 0.1, 0.0), (0.0, 0.0))
        self.assertTrue(path.complete)
        self.assertEqual(path.step(0.3, 10.0, 10.0), (0.0, 0.0))

    def test_fails_safe_on_timeout_regression_and_non_finite_odometry(self) -> None:
        timed_out = MappingPath(0.0, 0.0, 0.0, waypoint_timeout_s=1.0)
        self.assertEqual(timed_out.step(1.0, 0.0, 0.0), (0.0, 0.0))
        self.assertIn("timed out", timed_out.failure_reason or "")

        regressed = MappingPath(0.0, 0.0, 0.0)
        regressed.step(1.0, 0.0, 0.0)
        self.assertEqual(regressed.step(0.5, 0.0, 0.0), (0.0, 0.0))
        self.assertIn("backwards", regressed.failure_reason or "")

        invalid = MappingPath(0.0, 0.0, 0.0)
        self.assertEqual(invalid.step(0.1, float("nan"), 0.0), (0.0, 0.0))
        self.assertIn("non-finite", invalid.failure_reason or "")

    def test_rejects_unbounded_or_invalid_paths(self) -> None:
        invalid_paths = (
            lambda: MappingPath(0.0, 0.0, 0.0, offsets_m=()),
            lambda: MappingPath(0.0, 0.0, 0.0, offsets_m=((0.6, 0.0),)),
            lambda: MappingPath(0.0, 0.0, 0.0, waypoint_timeout_s=float("inf")),
        )
        for invalid_path in invalid_paths:
            with self.subTest(invalid_path=invalid_path), self.assertRaises(ValueError):
                invalid_path()


if __name__ == "__main__":
    unittest.main()
