"""Manual goal navigation with an online map; no autonomous goal generator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    share = get_package_share_directory('v550_navigation')
    base = get_package_share_directory('turn_on_wheeltec_robot')
    nav2 = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(directory, filename, arguments=None, condition=None):
        kwargs = {} if condition is None else {'condition': condition}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(directory, 'launch', filename)),
            launch_arguments=(arguments or {}).items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bringup_hardware', default_value='true',
                              description='Start chassis, EKF and the existing lidar driver'),
        DeclareLaunchArgument('use_wheeltec_imu', default_value='false',
                              description='Use the existing external Yesense IMU instead of board IMU'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_v550.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(share, 'config', 'slam_v550.yaml')),
        GroupAction([
            SetParameter(name='use_sim_time', value=use_sim_time),
            GroupAction(condition=IfCondition(LaunchConfiguration('bringup_hardware')), actions=[
                include(base, 'turn_on_wheeltec_robot.launch.py', {
                    'use_wheeltec_imu': LaunchConfiguration('use_wheeltec_imu')}),
                include(base, 'wheeltec_lidar.launch.py'),
                include(base, 'wheeltec_camera.launch.py',
                        condition=IfCondition(LaunchConfiguration('start_camera'))),
            ]),
            # SLAM is the sole publisher of map -> odom_combined and /map.
            # AMCL and a static map server must not run in this mode.
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', output='screen',
                 parameters=[LaunchConfiguration('slam_params_file'), {'use_sim_time': use_sim_time}],
                 condition=IfCondition(LaunchConfiguration('start_slam'))),
            include(nav2, 'navigation_launch.py', {
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true', 'use_composition': 'False',
            }, condition=IfCondition(LaunchConfiguration('start_nav2'))),
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', os.path.join(share, 'rviz', 'v550.rviz')],
                 condition=IfCondition(LaunchConfiguration('use_rviz'))),
        ]),
    ])
