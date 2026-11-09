"""Check with a real tf2 buffer when a map -> base_link read shows a new AMCL correction.

AMCL stamps each map -> odom it sends at its scan's stamp plus
transform_tolerance (nav2_amcl 1.3.12, laserReceived). GetLatestTransform reads
at the latest common time of the chain, about the robot_state_publisher time.
The localization Objectives accept a read only when its stamp is at least the
checked scan's stamp plus the tolerance. These tests feed tf2 the transforms
AMCL and robot_state_publisher would send, and check what such reads return.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time as RosTime
from tf2_ros import Buffer

PACKAGE = Path(__file__).resolve().parents[1]
NS = 1_000_000_000
START_NS = 1000 * NS
SCAN_PERIOD_NS = NS // 10
JOINT_STATE_PERIOD_NS = NS // 50
BEFORE, AFTER = 0.0, 1.0


def amcl_tolerance_ns():
    params = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    return int(params["amcl"]["ros__parameters"]["transform_tolerance"] * NS)


def edge(parent, child, ns, x):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = Time(sec=ns // NS, nanosec=ns % NS)
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.0
    return msg


def buffer_at(now_ns, update_scan_ns, tolerance_ns):
    """The buffer a stationary base leaves by now_ns.

    AMCL sends BEFORE for scans older than update_scan_ns and AFTER from then on.
    """
    buffer = Buffer()
    buffer.set_transform_static(edge("odom", "world", START_NS, 0.0), "static")
    for ns in range(START_NS, now_ns + 1, JOINT_STATE_PERIOD_NS):
        buffer.set_transform(edge("world", "base_link", ns, 0.0), "joint states")
    for scan_ns in range(START_NS, now_ns + 1, SCAN_PERIOD_NS):
        x = AFTER if scan_ns >= update_scan_ns else BEFORE
        buffer.set_transform(edge("map", "odom", scan_ns + tolerance_ns, x), "amcl")
    return buffer


def latest_read(buffer):
    """What GetLatestTransform returns: the stamp in ns and the x offset."""
    tf = buffer.lookup_transform("map", "base_link", RosTime())
    stamp = tf.header.stamp
    return stamp.sec * NS + stamp.nanosec, tf.transform.translation.x


def test_subtree_tolerance_matches_amcl():
    root = ET.parse(PACKAGE / "objectives/refine_localization_in_place_subtree.xml")
    port = root.getroot().find(
        "TreeNodesModel/SubTree/input_port[@name='amcl_transform_tolerance']"
    )
    assert int(float(port.get("default")) * NS) == amcl_tolerance_ns()


def test_a_read_soon_after_the_scan_shows_the_old_correction():
    """The failure the stamp check prevents: an early read scores the old estimate."""
    tolerance_ns = amcl_tolerance_ns()
    assert tolerance_ns > 3 * SCAN_PERIOD_NS
    update_ns = START_NS + 2 * NS
    check_scan_ns = update_ns + SCAN_PERIOD_NS
    stamp_ns, x = latest_read(
        buffer_at(check_scan_ns + 2 * SCAN_PERIOD_NS, update_ns, tolerance_ns)
    )
    assert stamp_ns < check_scan_ns + tolerance_ns
    assert x == BEFORE


def test_a_read_past_the_scan_plus_tolerance_shows_the_new_correction():
    tolerance_ns = amcl_tolerance_ns()
    update_ns = START_NS + 2 * NS
    check_scan_ns = update_ns + SCAN_PERIOD_NS
    accepted = 0
    end_ns = check_scan_ns + tolerance_ns + 5 * SCAN_PERIOD_NS
    for now_ns in range(check_scan_ns, end_ns, JOINT_STATE_PERIOD_NS):
        stamp_ns, x = latest_read(buffer_at(now_ns, update_ns, tolerance_ns))
        assert stamp_ns <= now_ns
        if stamp_ns >= check_scan_ns + tolerance_ns:
            assert x == AFTER, now_ns
            accepted += 1
    assert accepted > 0
