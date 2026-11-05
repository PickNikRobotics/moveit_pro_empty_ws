"""Localization, Nav2 and the planar base bridge, shared by mock, sim and hw.

Each mode's driver launch file includes this one and adds the lidar it has.

FRAMES
======
    map -> odom             AMCL, the localization correction.
    odom -> world           static identity, published here.
    world -> ... -> base_link
                            robot_state_publisher, through the planar joints.

Nothing else publishes a transform to base_link. The planar joints hold the
base pose in `world`: the true pose under mock and sim, wheel odometry on hw.

VELOCITY
========
    controller_server, behavior_server -> cmd_vel_nav -> velocity_smoother
        -> /base_cmd_vel -> planar_base_bridge.py -> planar_velocity_controller
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def generate_launch_description():
    share = get_package_share_directory("agilex_ranger_mini_mock")

    declared = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "params", "nav2_params.yaml"),
            description="Nav2 parameters for every node launched here.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(share, "maps", "suite_106.yaml"),
            description="Occupancy grid that map_server serves and AMCL localizes in.",
        ),
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="true",
            description="Seed AMCL at the pose in the parameter file.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="Nav2 log level."
        ),
    ]

    params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "yaml_filename": LaunchConfiguration("map"),
            "set_initial_pose": LaunchConfiguration("set_initial_pose"),
        },
        convert_types=True,
    )
    log_args = ["--ros-args", "--log-level", LaunchConfiguration("log_level")]

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="log",
            parameters=[params],
            arguments=log_args,
            remappings=list(remappings),
        )

    to_smoother = [("cmd_vel", "cmd_vel_nav")]
    nodes = [
        # Puts the URDF root under the odometry frame, so AMCL's correction moves the robot.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_odom_to_world",
            output="log",
            arguments=["--frame-id", "odom", "--child-frame-id", "world"],
        ),
        Node(
            package="agilex_ranger_mini_mock",
            executable="planar_base_bridge.py",
            name="planar_base_bridge",
            output="log",
        ),
        nav2_node("nav2_map_server", "map_server"),
        nav2_node("nav2_amcl", "amcl"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="log",
            parameters=[{"autostart": True, "node_names": LOCALIZATION_NODES}],
        ),
        nav2_node("nav2_controller", "controller_server", to_smoother),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server", to_smoother),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            to_smoother + [("cmd_vel_smoothed", "/base_cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="log",
            parameters=[{"autostart": True, "node_names": NAVIGATION_NODES}],
        ),
    ]
    return LaunchDescription(declared + nodes)
