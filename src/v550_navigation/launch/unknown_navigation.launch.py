"""Manual goal navigation with an online map; no autonomous goal generator."""
import os
import fcntl
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


# Keep the descriptor open until the launch process exits, including child shutdown.
# Locks automatically disappear on process exit; a leftover file never blocks a start.
_HARDWARE_LOCK = Path('/tmp') / f'wheeltec_v550_hardware_{os.getuid()}.lock'
_hardware_lock_handle = None


def _active_drivers():
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            executable = path.read_bytes().split(b'\0')[0].decode()
        except (OSError, UnicodeError):
            continue
        if Path(executable).name in ('wheeltec_robot_node', 'lslidar_driver_node', 'yesense_node_publisher'):
            found.append(f'{Path(executable).name} (PID {path.parent.name})')
    return found


def _acquire_hardware(context):
    global _hardware_lock_handle
    handle = _HARDWARE_LOCK.open('a+')
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('V550 已在运行，请先退出原启动终端；不要重复启动底盘、建图和导航。')
    drivers = _active_drivers()
    if drivers:
        handle.close()
        raise RuntimeError('检测到已有底盘/雷达驱动：' + ', '.join(drivers) +
                           '。请先退出旧程序；复用已有硬件时设置 bringup_hardware:=false，'
                           '并确保只有一套 EKF 和 SLAM 发布 TF。')
    _hardware_lock_handle = handle
    return []


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
        OpaqueFunction(function=_acquire_hardware,
                       condition=IfCondition(LaunchConfiguration('bringup_hardware'))),
        GroupAction([
            SetParameter(name='use_sim_time', value=use_sim_time),
            GroupAction(condition=IfCondition(LaunchConfiguration('bringup_hardware')), actions=[
                include(base, 'turn_on_wheeltec_robot.launch.py', {
                    'use_wheeltec_imu': LaunchConfiguration('use_wheeltec_imu')}),
                include(base, 'wheeltec_lidar.launch.py'),
                include(base, 'wheeltec_camera.launch.py',
                        condition=IfCondition(LaunchConfiguration('start_camera'))),
            ]),
            Node(package='v550_navigation', executable='normalize_scan',
                 name='v550_scan_normalizer', output='screen',
                 condition=IfCondition(LaunchConfiguration('start_slam'))),
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
