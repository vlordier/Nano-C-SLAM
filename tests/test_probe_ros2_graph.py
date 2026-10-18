"""Pure validation and child-state checks for the ROS-only DDS probe."""

from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from scripts.probe_ros2_graph import (
    ProbeFailure,
    _read_json,
    _stop_child,
    _wait_for_status,
    parse_node_names,
    require_unused_domain,
    resolve_executable,
    validate_domain_id,
    validate_exchange_evidence,
    validate_timeout,
)


class ProbeRos2GraphTests(unittest.TestCase):
    def test_timeout_must_be_finite_and_positive(self) -> None:
        for value in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_timeout(value)

    def test_domain_id_must_be_an_integer_in_supported_range(self) -> None:
        self.assertEqual(validate_domain_id(232), 232)
        for value in (-1, 233, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_domain_id(value)  # type: ignore[arg-type]

    def test_node_listing_ignores_ros_cli_warning_lines(self) -> None:
        self.assertEqual(
            parse_node_names("WARNING: daemon unavailable\n/cf1\n/cf3\n"),
            {"cf1", "cf3"},
        )

    def test_occupied_domain_is_rejected(self) -> None:
        with self.assertRaisesRegex(ProbeFailure, "already has participants"):
            require_unused_domain({"another_node"}, requested=True)

    def test_missing_executable_fails_explicitly(self) -> None:
        with (
            patch("scripts.probe_ros2_graph.shutil.which", return_value=None),
            self.assertRaisesRegex(FileNotFoundError, "not on PATH: ros2"),
        ):
            resolve_executable("ros2")

    def test_read_json_rejects_partial_and_non_object_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "status.json"
            self.assertIsNone(_read_json(path))
            path.write_text("{", encoding="utf-8")
            self.assertIsNone(_read_json(path))
            path.write_text(json.dumps(["not", "an object"]), encoding="utf-8")
            self.assertIsNone(_read_json(path))

    def test_status_waits_for_requested_transition(self) -> None:
        process = SimpleNamespace(poll=lambda: 0)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "status.json"
            path.write_text(json.dumps({"state": "ready"}), encoding="utf-8")
            with (
                patch("scripts.probe_ros2_graph.time.sleep", return_value=None),
                self.assertRaisesRegex(ProbeFailure, "exited before received"),
            ):
                _wait_for_status(
                    path,
                    process,
                    role="subscriber",
                    deadline=time.monotonic() + 1,
                    expected_state="received",
                )

    def test_status_timeout_is_bounded(self) -> None:
        process = SimpleNamespace(poll=lambda: None)
        with (
            tempfile.TemporaryDirectory() as temporary,
            self.assertRaisesRegex(ProbeFailure, "before the deadline"),
        ):
            _wait_for_status(
                Path(temporary) / "missing.json",
                process,
                role="publisher",
                deadline=time.monotonic() + 0.02,
                expected_state="published",
            )

    def test_owned_child_cleanup_signals_and_waits_for_child(self) -> None:
        class ChildProcess:
            returncode: int | None = None

            def poll(self) -> int | None:
                return self.returncode

            def wait(self, *, timeout: float) -> int:
                self.returncode = 0
                return 0

        with tempfile.TemporaryDirectory() as temporary:
            stop_path = Path(temporary) / "child.stop"
            process = ChildProcess()
            child: dict[str, Any] = {"process": process, "stop_path": stop_path}
            self.assertEqual(_stop_child(child), "stop-file")
            self.assertTrue(stop_path.exists())
            self.assertEqual(process.returncode, 0)

    def test_exchange_evidence_requires_exact_message_and_graph_nodes(self) -> None:
        subscriber = {
            "state": "received",
            "message": "nonce",
            "topic_type": "std_msgs/msg/String",
        }
        publisher = {
            "state": "published",
            "message": "nonce",
            "topic_type": "std_msgs/msg/String",
            "matched_topic_types": ["std_msgs/msg/String"],
        }
        validate_exchange_evidence(
            subscriber=subscriber,
            publisher=publisher,
            listed_nodes={"probe_sub", "probe_pub"},
            listed_topics="/probe/nonce [std_msgs/msg/String]",
            subscriber_name="probe_sub",
            publisher_name="probe_pub",
            topic="/probe/nonce",
            nonce="nonce",
        )
        with self.assertRaisesRegex(ProbeFailure, "exact nonce"):
            validate_exchange_evidence(
                subscriber={**subscriber, "message": "wrong"},
                publisher=publisher,
                listed_nodes={"probe_sub", "probe_pub"},
                listed_topics="/probe/nonce [std_msgs/msg/String]",
                subscriber_name="probe_sub",
                publisher_name="probe_pub",
                topic="/probe/nonce",
                nonce="nonce",
            )
        with self.assertRaisesRegex(ProbeFailure, "both fixture nodes"):
            validate_exchange_evidence(
                subscriber=subscriber,
                publisher=publisher,
                listed_nodes={"probe_sub"},
                listed_topics="/probe/nonce [std_msgs/msg/String]",
                subscriber_name="probe_sub",
                publisher_name="probe_pub",
                topic="/probe/nonce",
                nonce="nonce",
            )


if __name__ == "__main__":
    unittest.main()
