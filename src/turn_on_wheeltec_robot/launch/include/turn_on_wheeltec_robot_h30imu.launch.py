"""Compatibility entry for V550 with the existing external IMU driver."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turn_on_wheeltec_robot'),
            'launch', 'turn_on_wheeltec_robot.launch.py')),
        launch_arguments={'use_wheeltec_imu': 'true'}.items())])
