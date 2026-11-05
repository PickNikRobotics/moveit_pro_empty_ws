"""Processes that stay up for the whole session in simulation.

This is the sim config's `additional_driver_launch_file`: the mock config's
shared navigation stack. MuJoCo publishes the lidar scan on /scan itself, so
nothing replaces the mock lidar here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mock_share = get_package_share_directory("agilex_ranger_mini_mock")
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mock_share, "launch", "navigation.launch.py")
        )
    )
    return LaunchDescription([navigation])
