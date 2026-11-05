"""Processes that stay up for the whole session under mock hardware.

This is the mock config's `additional_driver_launch_file`: the shared
navigation stack, plus a lidar ray-cast through the map so AMCL and the
costmaps have a scan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agilex_ranger_mini_mock")
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "navigation.launch.py")
        )
    )
    mock_lidar = Node(
        package="agilex_ranger_mini_mock",
        executable="mock_lidar.py",
        name="mock_lidar",
        output="log",
    )
    return LaunchDescription([navigation, mock_lidar])
