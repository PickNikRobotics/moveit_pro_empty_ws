from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "standalone",
            default_value="false",
            description="Whether to start in standalone mode",
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cybertech_mock"),
                    "description",
                    "kuka_cybertech.urdf.xacro",
                ]
            ),
            " robot_name:=",
            "kuka_cybertech",
            " description_package:=",
            "kuka_kr16_support",
            " description_macro_file:=",
            "kr16_2_macro.xacro",
            " use_mock_hardware:=",
            "true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cybertech_mock"), "launch", "view_robot.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("standalone")),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("standalone")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
