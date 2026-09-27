"""V550 mecanum chassis, URDF and odometry. Sensor drivers are retained."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(share, 'launch')
    def include(name, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=(arguments or {}).items())
    return LaunchDescription([
        DeclareLaunchArgument('use_wheeltec_imu', default_value='false'),
        include('include/base_serial_h30imu.launch.py', {
            'use_wheeltec_imu': LaunchConfiguration('use_wheeltec_imu')}),
        include('robot_mode_description_minibot.launch.py'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_link',
             arguments=['--z', '0.02027', '--frame-id', 'base_footprint',
                        '--child-frame-id', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_gyro',
             arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'gyro_link']),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             parameters=[os.path.join(share, 'config', 'imu.yaml')]),
        include('wheeltec_ekf.launch.py'),
    ])
