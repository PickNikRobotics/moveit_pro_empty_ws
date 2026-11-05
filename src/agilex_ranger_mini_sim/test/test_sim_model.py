"""Check that the MuJoCo model agrees with the URDF and the mock config, without launching."""

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]
MJCF = PACKAGE / "mjcf"
PLANAR_JOINTS = ["linear_x_joint", "linear_y_joint", "rotational_yaw_joint"]
URDF_TO_MJCF_TYPE = {"prismatic": "slide", "revolute": "hinge", "continuous": "hinge"}


@pytest.fixture(scope="module")
def mock_share():
    return Path(get_package_share_directory("agilex_ranger_mini_mock"))


@pytest.fixture(scope="module")
def urdf(mock_share):
    doc = xacro.process_file(
        str(mock_share / "description/agilex_ranger_mini.urdf.xacro"),
        mappings={
            "hardware_xacro": str(
                PACKAGE / "description/agilex_ranger_mini_sim.ros2_control.xacro"
            )
        },
    )
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope="module")
def robot_mjcf():
    return ET.parse(MJCF / "agilex_ranger_mini.xml").getroot()


def test_generated_files_are_current():
    result = subprocess.run(
        [sys.executable, str(PACKAGE / "scripts/generate_mjcf.py"), "--check"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_ros2_control_uses_mujoco(urdf):
    plugin = urdf.find("ros2_control/hardware/plugin").text
    assert plugin == "picknik_mujoco_ros/MujocoSystem"
    params = {
        p.get("name"): p.text for p in urdf.findall("ros2_control/hardware/param")
    }
    assert params["mujoco_model_package"] == "agilex_ranger_mini_sim"
    assert (PACKAGE / params["mujoco_model"]).is_file()
    assert params["base_link_name"] == "base_link"


def test_every_urdf_joint_is_in_the_mjcf(urdf, robot_mjcf):
    mjcf_joints = {j.get("name"): j.get("type") for j in robot_mjcf.iter("joint")}
    for joint in urdf.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        name = joint.get("name")
        assert mjcf_joints.get(name) == URDF_TO_MJCF_TYPE[joint.get("type")], name


def test_actuators_are_the_command_joints(urdf, robot_mjcf):
    commanded = [
        j.get("name")
        for j in urdf.findall("ros2_control/joint")
        if j.find("command_interface") is not None
    ]
    actuators = robot_mjcf.findall("actuator/velocity")
    assert [a.get("name") for a in actuators] == commanded == PLANAR_JOINTS
    assert all(a.get("joint") == a.get("name") for a in actuators)


def test_keyframe_matches_the_mock_spawn(mock_share):
    scene = ET.parse(MJCF / "scene.xml").getroot()
    key = scene.find("keyframe/key[@name='default']")
    qpos = [float(v) for v in key.get("qpos").split()]
    initial = yaml.safe_load((mock_share / "config/initial_positions.yaml").read_text())
    assert qpos[:3] == [initial[n] for n in PLANAR_JOINTS]
    # Three planar joints, then a steering and a wheel joint per corner.
    assert len(qpos) == 3 + 2 * 4


def test_model_compiles_and_the_lidar_sees_walls_not_the_robot():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(MJCF / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ranges = data.sensordata[data.sensordata >= 0]
    assert len(ranges) > 0.9 * model.nsensor
    # The nearest return at spawn is a wall, well beyond the robot's own outline.
    assert ranges.min() > 0.5


def test_base_drives_and_stops_at_a_wall():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(MJCF / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = [0.5, 0.0, 0.0]
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert abs(data.qvel[0] - 0.5) < 0.01
    assert abs(data.qpos[1] - 12.75) < 1e-6
    # Drive west for long enough to reach the room's west wall: the base must not pass it.
    data.ctrl[:] = [-0.5, 0.0, 0.0]
    for _ in range(int(20.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert data.qpos[0] > 0.3
