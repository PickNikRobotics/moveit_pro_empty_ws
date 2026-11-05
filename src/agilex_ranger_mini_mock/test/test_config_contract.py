"""Check that the mock config's files agree with each other, without launching.

The names and poses below appear in several files that nothing else compares:
the URDF's ros2_control block, the controller YAML, the SRDF, config.yaml, the
Nav2 parameters, the waypoints and the Objectives.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
PLANAR_JOINTS = ["linear_x_joint", "linear_y_joint", "rotational_yaw_joint"]
CORNERS = ["fl", "fr", "rl", "rr"]
STATE_JOINTS = [f"{c}_steering_joint" for c in CORNERS] + [
    f"{c}_wheel_joint" for c in CORNERS
]


def load_yaml(relative):
    return yaml.safe_load((PACKAGE / relative).read_text())


@pytest.fixture(scope="module")
def urdf():
    doc = xacro.process_file(
        str(PACKAGE / "description/agilex_ranger_mini.urdf.xacro"),
        mappings={
            "hardware_xacro": str(
                PACKAGE / "description/agilex_ranger_mini_mock.ros2_control.xacro"
            ),
            "initial_positions_file": str(PACKAGE / "config/initial_positions.yaml"),
        },
    )
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(PACKAGE / "config/moveit/agilex_ranger_mini.srdf").getroot()


@pytest.fixture(scope="module")
def controllers():
    return load_yaml("config/control/agilex_ranger_mini.ros2_control.yaml")


def ros2_control_joints(urdf):
    """Map each ros2_control joint to its command interface names."""
    block = urdf.find("ros2_control")
    return {
        j.get("name"): [c.get("name") for c in j.findall("command_interface")]
        for j in block.findall("joint")
    }


def test_urdf_has_one_root_at_world(urdf):
    links = {link.get("name") for link in urdf.findall("link")}
    children = {j.find("child").get("link") for j in urdf.findall("joint")}
    assert links - children == {"world"}


def test_ros2_control_joint_vocabulary(urdf):
    joints = ros2_control_joints(urdf)
    assert list(joints) == PLANAR_JOINTS + STATE_JOINTS
    for name in PLANAR_JOINTS:
        assert joints[name] == ["velocity"]
    for name in STATE_JOINTS:
        assert joints[name] == []
    urdf_joints = {j.get("name") for j in urdf.findall("joint")}
    assert set(joints) <= urdf_joints


def test_controllers_claim_the_planar_joints(controllers):
    cm = controllers["controller_manager"]["ros__parameters"]
    declared = {k for k, v in cm.items() if isinstance(v, dict) and "type" in v}
    jsb = controllers["joint_state_broadcaster"]["ros__parameters"]
    assert jsb["joints"] == PLANAR_JOINTS + STATE_JOINTS
    for name in ["planar_velocity_controller", "joint_trajectory_controller"]:
        assert controllers[name]["ros__parameters"]["joints"] == PLANAR_JOINTS
    for name in ["joint_velocity_controller", "velocity_force_controller"]:
        params = controllers[name]["ros__parameters"]
        assert params["planning_group_name"] == "mobile_base"
        assert params["command_interfaces"] == ["velocity"]
        assert len(params["max_joint_velocity"]) == len(PLANAR_JOINTS)
    config = load_yaml("config/config.yaml")["ros2_control"]
    managed = set(config["controllers_active_at_startup"]) | set(
        config["controllers_inactive_at_startup"]
    )
    assert managed == declared


def test_srdf_groups(srdf):
    groups = {g.get("name"): g for g in srdf.findall("group")}
    chain = groups["mobile_base"].find("chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("world", "base_link")
    ee = srdf.find("end_effector")
    assert ee.get("parent_link") == "base_link"
    assert ee.get("parent_group") == "mobile_base"
    kinematics = load_yaml("config/moveit/pose_ik.yaml")
    assert set(kinematics) == {"mobile_base"}
    assert (
        load_yaml("config/config.yaml")["moveit_params"]["joint_group_name"]
        == "mobile_base"
    )


def test_jog_files_name_declared_controllers(controllers):
    for relative in ["config/moveit/joint_jog.yaml", "config/moveit/pose_jog.yaml"]:
        jog = load_yaml(relative)
        assert jog["planning_groups"] == ["mobile_base"]
        assert len(jog["controllers"]) == len(jog["planning_groups"])
        for name in jog["controllers"]:
            assert name in controllers


def test_spawn_pose_agrees_everywhere(srdf):
    initial = load_yaml("config/initial_positions.yaml")
    spawn = [initial[name] for name in PLANAR_JOINTS]

    state = next(s for s in srdf.findall("group_state") if s.get("name") == "Spawn")
    assert [
        float(state.find(f"joint[@name='{n}']").get("value")) for n in PLANAR_JOINTS
    ] == spawn

    amcl = load_yaml("params/nav2_params.yaml")["amcl"]["ros__parameters"]
    seed = amcl["initial_pose"]
    assert [seed["x"], seed["y"], seed["yaw"]] == spawn

    waypoints = load_yaml("waypoints/agilex_ranger_mini_waypoints.yaml")
    point = next(w for w in waypoints if w["name"] == "Spawn")
    positions = dict(
        zip(point["joint_state"]["name"], point["joint_state"]["position"])
    )
    assert [positions[n] for n in PLANAR_JOINTS] == spawn


def test_spawn_is_on_free_floor():
    """The spawn cell and its footprint must be free in the map AMCL localizes in."""
    pil = pytest.importorskip("PIL.Image")
    meta = load_yaml("maps/suite_106.yaml")
    grey = pil.open(PACKAGE / "maps" / meta["image"]).convert("L")
    initial = load_yaml("config/initial_positions.yaml")
    res = meta["resolution"]
    for dx in (-0.4, 0.0, 0.4):
        for dy in (-0.3, 0.0, 0.3):
            col = int((initial["linear_x_joint"] + dx - meta["origin"][0]) / res)
            row = (
                grey.height
                - 1
                - int((initial["linear_y_joint"] + dy - meta["origin"][1]) / res)
            )
            occupancy = (255 - grey.getpixel((col, row))) / 255.0
            assert occupancy < meta["free_thresh"]


def test_nav2_planners_and_controller():
    params = load_yaml("params/nav2_params.yaml")
    planner = params["planner_server"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased", "Smac"]
    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert planner["Smac"]["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow["motion_model"] == "Omni"
    amcl = params["amcl"]["ros__parameters"]
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"


def test_objectives_parse_and_name_real_controllers(controllers):
    objectives = sorted((PACKAGE / "objectives").glob("*.xml"))
    assert objectives
    for path in objectives:
        root = ET.parse(path).getroot()
        for node in root.iter():
            for key in (
                "activate_controllers",
                "controller_names",
                "joint_trajectory_controller_name",
            ):
                for name in filter(None, (node.get(key) or "").split(";")):
                    if not name.startswith("{"):
                        assert name in controllers, f"{path.name}: {name}"
            if node.get("planner_id"):
                planner = load_yaml("params/nav2_params.yaml")["planner_server"]
                assert (
                    node.get("planner_id")
                    in planner["ros__parameters"]["planner_plugins"]
                )


def test_lidar_mount_matches_mock_lidar(urdf):
    joint = next(j for j in urdf.findall("joint") if j.get("name") == "laser_joint")
    x, y, _ = (float(v) for v in joint.find("origin").get("xyz").split())
    # The mock lidar node assumes the default offset, a lidar centred on the base.
    assert math.isclose(x, 0.0) and math.isclose(y, 0.0)
