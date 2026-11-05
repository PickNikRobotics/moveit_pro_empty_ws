"""Check the hw config's description and its driver bridge, without hardware."""

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

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
