"""Processes that stay up for the whole session on the physical Ranger Mini.

UNTESTED ON HARDWARE. This is the hw config's `additional_driver_launch_file`:

  ranger_base_node       AgileX's CAN driver. It takes a body twist and
                         publishes wheel odometry; its topics are moved under
                         /agilex_ranger_mini/driver so nothing else drives it.
  ranger_driver_bridge   the ros2_control topic block <-> the driver.
  lidar                  the launch file named by `lidar_launch_file`, if any.
  navigation             the mock config's shared Nav2 stack, with AMCL
                         seeded at the boot pose and real-odometry noise.

MoveIt Pro includes this file without arguments, so set each value by
editing its `default_value` below. The CAN interface must be up on the host
first; see the package README.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

DRIVER_NS = "/agilex_ranger_mini/driver"
# Nav2's default AMCL odometry noise; the shared file assumes exact odometry.
HW_ODOM_ALPHA = "0.2"


def amcl_hw_rewrites(initial_positions):
    """Nav2 parameter rewrites, by full path, that adapt AMCL to the robot.

    @param initial_positions The hw boot values of the planar joints.
    @return Rewrites that seed AMCL at the boot pose and raise the odometry noise.
    """
    amcl = "amcl.ros__parameters."
    rewrites = {
        amcl + "set_initial_pose": "true",
        amcl + "initial_pose.x": str(float(initial_positions["linear_x_joint"])),
        amcl + "initial_pose.y": str(float(initial_positions["linear_y_joint"])),
        amcl
        + "initial_pose.yaw": str(float(initial_positions["rotational_yaw_joint"])),
    }
    for i in range(1, 6):
        rewrites[f"{amcl}alpha{i}"] = HW_ODOM_ALPHA
    return rewrites


def generate_launch_description():
    mock_share = get_package_share_directory("agilex_ranger_mini_mock")
    hw_share = get_package_share_directory("agilex_ranger_mini_hw")
    with open(os.path.join(hw_share, "config", "initial_positions.yaml")) as f:
        initial_positions = yaml.safe_load(f)
    lidar_launch_file = LaunchConfiguration("lidar_launch_file")
    no_lidar = PythonExpression(["'", lidar_launch_file, "' == ''"])

    declared = [
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="SocketCAN interface of the base.",
        ),
        DeclareLaunchArgument(
            "lidar_launch_file",
            default_value="",
            description=(
                "Launch file of the navigation lidar driver. It must publish a "
                "sensor_msgs/LaserScan on /scan, stamped in laser_link."
            ),
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(mock_share, "maps", "suite_106.yaml"),
            description="Occupancy grid of the site, from a mapping run.",
        ),
    ]

    driver = Node(
        package="ranger_base",
        executable="ranger_base_node",
        name="ranger_base_node",
        output="both",
        parameters=[
            {
                "port_name": LaunchConfiguration("can_port"),
                "robot_model": "ranger_mini_v3",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "odom_topic_name": f"{DRIVER_NS}/odom",
                # robot_state_publisher owns the base pose through the planar joints.
                "publish_odom_tf": False,
            }
        ],
        remappings=[("/cmd_vel", f"{DRIVER_NS}/cmd_vel")],
    )
    bridge = Node(
        package="agilex_ranger_mini_hw",
        executable="ranger_driver_bridge.py",
        name="ranger_driver_bridge",
        output="both",
        parameters=[
            {
                "driver_cmd_vel_topic": f"{DRIVER_NS}/cmd_vel",
                "driver_odom_topic": f"{DRIVER_NS}/odom",
            }
        ],
    )
    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(lidar_launch_file),
        condition=UnlessCondition(no_lidar),
    )
    no_lidar_warning = LogInfo(
        condition=IfCondition(no_lidar),
        msg="agilex_ranger_mini_hw: no lidar_launch_file given, so AMCL and the costmaps get no scan.",
    )
    nav2_params = RewrittenYaml(
        source_file=os.path.join(mock_share, "params", "nav2_params.yaml"),
        root_key="",
        param_rewrites=amcl_hw_rewrites(initial_positions),
        convert_types=True,
    )
    navigation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mock_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "map": LaunchConfiguration("map"),
        }.items(),
    )
    return LaunchDescription(
        declared + [driver, bridge, lidar, no_lidar_warning, navigation]
    )
