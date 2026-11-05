"""Check the hw config's description and its driver bridge, without hardware."""

import importlib.util
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

PACKAGE = Path(__file__).resolve().parents[1]
PLANAR_JOINTS = ["linear_x_joint", "linear_y_joint", "rotational_yaw_joint"]


def load_bridge():
    path = PACKAGE / "scripts/ranger_driver_bridge.py"
    spec = importlib.util.spec_from_file_location("ranger_driver_bridge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load_bridge()


@pytest.fixture(scope="module")
def urdf():
    """Expand with every urdf_param the hw config ends up with, inherited ones included."""
    mock = Path(get_package_share_directory("agilex_ranger_mini_mock"))
    params = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    mappings = {"mujoco_model": "mjcf/scene.xml", "mujoco_viewer": "false"}
    for entry in params["hardware"]["robot_description"]["urdf_params"]:
        for key, value in entry.items():
            if isinstance(value, dict):
                value = str(PACKAGE / value["path"])
            mappings[key] = value
    doc = xacro.process_file(
        str(mock / "description/agilex_ranger_mini.urdf.xacro"), mappings=mappings
    )
    return ET.fromstring(doc.toxml())


def test_hw_uses_the_topic_interface(urdf):
    plugin = urdf.find("ros2_control/hardware/plugin").text
    assert plugin == "joint_state_topic_hardware_interface/JointStateTopicSystem"
    params = {
        p.get("name"): p.text for p in urdf.findall("ros2_control/hardware/param")
    }
    assert float(params["trigger_joint_command_threshold"]) < 0


def test_commanded_joints_come_first(urdf):
    joints = urdf.findall("ros2_control/joint")
    commanded = [j.find("command_interface") is not None for j in joints]
    assert [j.get("name") for j in joints[:3]] == PLANAR_JOINTS
    assert commanded == [True] * 3 + [False] * (len(joints) - 3)


def test_hw_boots_at_the_odometry_origin(urdf):
    for joint in urdf.findall("ros2_control/joint")[:3]:
        position = joint.find("state_interface[@name='position']/param")
        assert float(position.text) == 0.0


class FakeJointState:
    def __init__(self, name, velocity):
        self.name, self.velocity = name, velocity


def test_command_pairs_the_first_names_with_the_velocities():
    names = PLANAR_JOINTS + ["fl_steering_joint", "fl_wheel_joint"]
    msg = FakeJointState(names, [0.1, 0.2, 0.3])
    assert bridge.commanded_planar_velocity(msg) == (0.1, 0.2, 0.3)
    assert bridge.commanded_planar_velocity(FakeJointState(names, [0.1])) is None


@pytest.mark.parametrize("yaw", [0.0, 1.0, -2.0, math.pi])
def test_twist_round_trip(yaw):
    world = bridge.body_to_world(0.4, -0.1, 0.2, yaw)
    assert bridge.world_to_body(*world, yaw) == pytest.approx((0.4, -0.1, 0.2))


def test_unwrap_keeps_yaw_continuous():
    yaw = None
    for angle in [3.0, -3.1, -2.9, 3.1]:
        yaw = bridge.unwrap(yaw, angle)
    assert yaw == pytest.approx(3.1)
    assert bridge.unwrap(3.0, -3.1) == pytest.approx(-3.1 + 2.0 * math.pi)


def test_yaw_from_quaternion():
    assert bridge.yaw_from_quaternion(math.sin(0.35), math.cos(0.35)) == pytest.approx(
        0.7
    )


def load_planar_base_bridge():
    path = PACKAGE.parent / "agilex_ranger_mini_mock/scripts/planar_base_bridge.py"
    spec = importlib.util.spec_from_file_location("planar_base_bridge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LIMITS = (2.0, 3.259)
ODOM_PERIOD = 1.0 / 50.0  # The driver's default odometry rate; any rate works.


def published_headings(yaw, wz, count):
    """Headings the bridge published while the base turned at wz, newest last."""
    return [yaw - wz * ODOM_PERIOD * k for k in reversed(range(count))]


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_command_stops_the_base(bad):
    assert bridge.driver_twist((bad, 0.1, 0.1), [0.3], *LIMITS) == (0.0, 0.0, 0.0)
    assert bridge.driver_twist((0.1, 0.1, 0.1), [0.0, bad], *LIMITS) == (0.0, 0.0, 0.0)
    assert bridge.driver_twist((0.1, 0.1, 0.1), [], *LIMITS) == (0.0, 0.0, 0.0)


def test_huge_finite_command_stays_finite():
    out = bridge.driver_twist((1e308, 1e308, 0.5), [0.0], *LIMITS)
    assert math.hypot(out[0], out[1]) == pytest.approx(LIMITS[0])
    out = bridge.driver_twist((1.7e308, 1.7e308, 0.5), [math.pi / 4], *LIMITS)
    assert all(math.isfinite(v) for v in out)


@pytest.mark.parametrize("lag", [0, 1, 2, 24])
@pytest.mark.parametrize("axis", ["forward", "sideways"])
def test_single_axis_command_reaches_the_driver_exactly(lag, axis):
    """A command built from any published heading keeps exactly one linear axis.

    The driver drops the turn rate whenever vy is not exactly zero, so
    forward-and-turn must arrive with vy == 0. lag counts odometry messages
    between the heading the command used and the newest one.
    """
    nav2 = load_planar_base_bridge()
    rng = random.Random(lag)
    for _ in range(500):
        yaw = rng.uniform(-math.pi, math.pi)
        speed = rng.choice((-1.0, 1.0)) * rng.uniform(1e-3, 0.5)
        wz = rng.uniform(-LIMITS[1], LIMITS[1])
        headings = published_headings(yaw, wz, 25)
        built_with = headings[-1 - lag]
        body = (speed, 0.0) if axis == "forward" else (0.0, speed)
        rates = nav2.body_to_world(*body, wz, built_with)
        out = bridge.driver_twist(rates, headings, *LIMITS)
        assert out == pytest.approx((*body, wz))
        assert out[1 if axis == "forward" else 0] == 0.0


def test_heading_half_a_turn_back_never_reverses_the_command():
    """The guard stops the search a quarter turn from the newest heading.

    With the history [pi, 0.7] the newest heading does not zero an axis, so the
    loop reaches pi. There the world x command (0.3, 0, 0) would read as
    (-0.3, 0): the reverse of the command. Without the guard this test fails.
    """
    nav2 = load_planar_base_bridge()
    rates = (0.3, 0.0, 0.0)
    out = bridge.driver_twist(rates, [math.pi, 0.7], *LIMITS)
    assert out == pytest.approx(nav2.world_to_body(*rates, 0.7))
    assert out[0] > 0.0 and out[1] != 0.0  # Neither reversed nor snapped to an axis.
    # The newest heading matches first, so the old one is never reached.
    out = bridge.driver_twist(rates, [0.0, math.pi], *LIMITS)
    assert out == pytest.approx(nav2.world_to_body(*rates, math.pi))
    # Just over a quarter turn the axes swap: forward must not read as sideways.
    newest = math.pi / 2 + 1.6
    out = bridge.driver_twist(rates, [math.pi / 2, newest], *LIMITS)
    assert out == pytest.approx(nav2.world_to_body(*rates, newest))
    assert out[0] != 0.0 and out[1] != 0.0


def test_world_frame_command_can_match_an_older_heading():
    """Pins retained behaviour: a world jog is read as built from a retained heading.

    A joint jog is not built from any heading. If an older heading in the
    history (the boot heading, after the base first turns) leaves a body axis at
    rounding level, the command is sent along that older heading.
    """
    nav2 = load_planar_base_bridge()
    rates = (0.3, 0.0, 0.0)
    out = bridge.driver_twist(rates, [0.0, 0.1], *LIMITS)
    assert out == (0.3, 0.0, 0.0)  # Along the older heading, vy exactly zero.
    assert out != pytest.approx(nav2.world_to_body(*rates, 0.1))
    # Once the older heading has left the history, the newest one is used.
    out = bridge.driver_twist(rates, [0.05, 0.1], *LIMITS)
    assert out == pytest.approx(nav2.world_to_body(*rates, 0.1))


def test_real_sideways_and_forward_motion_is_kept():
    """No heading hides a command that really has both linear parts."""
    nav2 = load_planar_base_bridge()
    rng = random.Random(7)
    for _ in range(500):
        yaw = rng.uniform(-math.pi, math.pi)
        vx, vy = rng.uniform(0.05, 0.5), rng.choice((-1, 1)) * rng.uniform(1e-3, 0.5)
        wz = rng.uniform(-1.0, 1.0)
        headings = published_headings(yaw, wz, 25)
        rates = nav2.body_to_world(vx, vy, wz, headings[-1])
        out = bridge.driver_twist(rates, headings, *LIMITS)
        assert out == pytest.approx((vx, vy, wz))


def test_diagonal_command_is_rotated_with_the_newest_heading():
    """Both parts are kept, but the original body direction is not recovered.

    A diagonal command built from an older heading matches no heading, so it is
    read with the newest one. The direction turns by the heading change.
    """
    nav2 = load_planar_base_bridge()
    headings = published_headings(0.4, 1.0, 25)
    for lag in (0, 3, 24):
        rates = nav2.body_to_world(0.3, -0.2, 1.0, headings[-1 - lag])
        out = bridge.driver_twist(rates, headings, *LIMITS)
        assert out == pytest.approx(nav2.world_to_body(*rates, headings[-1]))
        assert out[0] != 0.0 and out[1] != 0.0
        angle = math.atan2(out[1], out[0]) - math.atan2(-0.2, 0.3)
        assert angle == pytest.approx(-(headings[-1] - headings[-1 - lag]))


def test_driver_twist_caps_speed_and_turn():
    vx, vy, wz = bridge.driver_twist((3.0, 4.0, 9.0), [0.0], *LIMITS)
    assert math.hypot(vx, vy) == pytest.approx(LIMITS[0])
    assert wz == LIMITS[1]


class FakeClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self.ns


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def make_node():
    """A bridge with fake time and output, without ROS."""
    node = bridge.RangerDriverBridge.__new__(bridge.RangerDriverBridge)
    node._clock_fake = FakeClock()
    node.get_clock = lambda: node._clock_fake
    node.get_logger = lambda: type("L", (), {"warn": lambda *a, **k: None})()
    node._twist_pub, node._state_pub = FakePublisher(), FakePublisher()
    node._timeout_ns, node._odom_timeout_ns = int(0.25e9), int(0.5e9)
    node._period_ns, node._limits = int(0.02e9), LIMITS
    node._headings = bridge.deque(maxlen=25)
    node._yaw = node._last_command_ns = node._last_odom_ns = None
    node._last_publish_ns = node._last_twist = None
    return node


def odom(yaw=0.0, x=0.0):
    t = type("T", (), {})
    msg = t()
    msg.header = t()
    msg.header.stamp = Time()
    msg.pose = t()
    msg.pose.pose = t()
    msg.pose.pose.position = t()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, 0.0
    msg.pose.pose.orientation = t()
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist = t()
    msg.twist.twist = t()
    msg.twist.twist.linear = t()
    msg.twist.twist.linear.x = msg.twist.twist.linear.y = 0.0
    msg.twist.twist.angular = t()
    msg.twist.twist.angular.z = 0.0
    return msg


def test_rate_limit_applies_to_stops_too():
    """Moving twists and stops pass once per period; the first stop passes at once."""
    node = make_node()
    node._on_odom(odom())
    sent = []
    clock = node._clock_fake
    node._twist_pub.publish = lambda m: sent.append(
        (clock.ns // 1_000_000, m.linear.x, m.linear.y, m.angular.z)
    )
    # Commands arrive every 5 ms: three moving, then stops. The first stop at
    # 15 ms is inside the period of the 0 ms twist, and still passes.
    for rates in [(0.1, 0.0, 0.0)] * 3 + [(0.0, 0.0, 0.0)] * 10:
        node._on_command(FakeJointState(PLANAR_JOINTS, list(rates)))
        clock.ns += 5_000_000
    assert sent == [
        (0, 0.1, 0.0, 0.0),
        (15, 0.0, 0.0, 0.0),
        (35, 0.0, 0.0, 0.0),
        (55, 0.0, 0.0, 0.0),
    ]


def test_non_finite_odometry_is_dropped():
    node = make_node()
    node._on_odom(odom(yaw=0.2))
    node._on_odom(odom(yaw=math.nan))
    assert node._yaw == pytest.approx(0.2)
    assert list(node._headings) == [pytest.approx(0.2)]
    assert len(node._state_pub.sent) == 1


def test_amcl_rewrites_name_existing_float_parameters():
    """A typo in a rewrite path, or an integer, would break AMCL only on hw."""
    spec = importlib.util.spec_from_file_location(
        "hw_drivers", PACKAGE / "launch/hw_drivers.launch.py"
    )
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    mock = PACKAGE.parent / "agilex_ranger_mini_mock"
    nav2 = yaml.safe_load((mock / "params/nav2_params.yaml").read_text())
    for path, value in launch.amcl_hw_rewrites({j: 0 for j in PLANAR_JOINTS}).items():
        node = nav2
        for key in path.split("."):
            assert key in node, path
            node = node[key]
        if not isinstance(node, bool):
            assert "." in value, f"{path} must be written as a float"


def test_bridge_ceilings_match_the_urdf(urdf):
    import rclpy

    rclpy.init()
    try:
        node = bridge.RangerDriverBridge()
        defaults = node._limits
        node.destroy_node()
    finally:
        rclpy.shutdown()
    limits = {
        j.get("name"): float(j.find("limit").get("velocity"))
        for j in urdf.findall("joint")
        if j.get("name") in PLANAR_JOINTS
    }
    assert defaults[0] == limits["linear_x_joint"] == limits["linear_y_joint"]
    assert defaults[1] == limits["rotational_yaw_joint"]
    assert defaults == LIMITS


def test_hw_waypoints_stay_at_the_odometry_origin():
    """Inherited map-place waypoints would drive blind from the power-on spot."""
    params = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    entry = params["objectives"]["waypoints_file"]
    assert entry["package_name"] == "agilex_ranger_mini_hw"
    waypoints = yaml.safe_load((PACKAGE / entry["relative_path"]).read_text())
    assert [w["name"] for w in waypoints] == ["Spawn"]
    for waypoint in waypoints:
        assert set(waypoint["joint_state"]["position"]) == {0.0}
