"""Small transformations for the upstream ros_gz_bridge YAML configuration."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

CF1_ODOMETRY_ROS_TOPIC = "/cf1/odom"
CF1_TRUTH_ODOMETRY_ROS_TOPIC = "/nano_c_slam_failure_probe/cf1/odom_truth"


def split_cf1_odometry_bridge(
    entries: Sequence[Mapping[str, object]],
) -> tuple[list[dict[str, object]], dict[str, object], dict[str, object]]:
    """Separate mission odometry from other bridges and add a truth mirror.

    The returned odometry entry can be stopped independently while the regular
    bridge continues carrying simulator clock and control topics. The mirror
    is an independent, read-only ROS topic for post-exit state observation.
    """
    copied_entries = [dict(entry) for entry in entries]
    odometry_entries = [
        entry
        for entry in copied_entries
        if entry.get("ros_topic_name") == CF1_ODOMETRY_ROS_TOPIC
    ]
    if len(odometry_entries) != 1:
        raise ValueError("bridge configuration must contain exactly one /cf1/odom")

    odometry = odometry_entries[0]
    if (
        odometry.get("direction") != "GZ_TO_ROS"
        or not isinstance(odometry.get("gz_topic_name"), str)
        or not isinstance(odometry.get("ros_type_name"), str)
        or not isinstance(odometry.get("gz_type_name"), str)
    ):
        raise ValueError("/cf1/odom bridge entry is incomplete or not GZ_TO_ROS")

    truth = dict(odometry)
    truth["ros_topic_name"] = CF1_TRUTH_ODOMETRY_ROS_TOPIC
    main_entries = [
        entry
        for entry in copied_entries
        if entry.get("ros_topic_name") != CF1_ODOMETRY_ROS_TOPIC
    ]
    return main_entries, odometry, truth
