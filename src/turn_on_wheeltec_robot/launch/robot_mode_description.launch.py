"""V550 mecanum only. The URDF owns laser and camera_link transforms."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory('wheeltec_robot_urdf'),
                'urdf', 'V550_mec_robot.urdf').read_text()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher'),
    ])
