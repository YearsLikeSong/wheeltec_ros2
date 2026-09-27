#!/usr/bin/env python3
"""One-time, backed-up V550 cleanup. Run backup, then configure, then prune."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
BACKUP = ROOT / 'backups/v550_before_cleanup'
REMOVE = [
    'aruco_ros-humble-devel', 'ultralytics_ros2', 'wheeltec_multi',
    'wheeltec_path_follow', 'wheeltec_robot_kcf', 'wheeltec_robot_rrt2',
    'wheeltec_rrt_msg', 'wheeltec_robot_rtab',
    'wheeltec_robot_slam/orb_slam_2_ros-ros2',
    'wheeltec_robot_slam/openslam_gmapping',
    'wheeltec_robot_slam/slam_gmapping',
    'wheeltec_robot_slam/wheeltec_cartographer',
]
DRIVERS = [
    'wheeltec_lidar_ros2', 'ros2_astra_camera-master', 'usb_cam-ros2',
    'wheeltec_gps', 'wheeltec_imu', 'wheeltec_mic', 'wheeltec_aiui',
    'wheeltec_bodyreader', 'tts_make_ros2', 'wheeltec_joy',
    'wheeltec_robot_keyboard', 'auto_recharge_ros2', 'web_video_server-ros2',
]


def hashes():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for name in DRIVERS for p in sorted((ROOT / 'src' / name).rglob('*'))
            if p.is_file()}


def backup():
    BACKUP.mkdir(parents=True, exist_ok=True)
    (ROOT / 'backups/COLCON_IGNORE').touch()
    archive = BACKUP / 'original_sources.tar.gz'
    if archive.exists():
        raise SystemExit('Backup already exists; refusing to overwrite it.')
    with tarfile.open(str(archive) + '.partial', 'w:gz', compresslevel=1) as out:
        for name in ('src', 'colcon.meta', 'robot_mode_description.launch.py'):
            path = ROOT / name
            if path.exists():
                out.add(path, arcname=name)
    Path(str(archive) + '.partial').rename(archive)
    (BACKUP / 'peripheral_sha256.json').write_text(json.dumps(hashes(), indent=2))
    print(f'Backup complete: {archive}', flush=True)


def configure():
    assert (BACKUP / 'original_sources.tar.gz').exists(), 'Create the backup first'
    config = ROOT / 'src/v550_navigation/config'
    config.mkdir(parents=True, exist_ok=True)
    nav = yaml.safe_load((ROOT / 'src/navigation2-humble/nav2_bringup/params/nav2_params.yaml').read_text())
    def patch_frames(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    value[key] = False
                elif key in ('base_frame_id', 'robot_base_frame'):
                    value[key] = 'base_footprint'
                elif key in ('odom_frame_id', 'odom_topic') or (key == 'global_frame' and child == 'odom'):
                    value[key] = 'odom_combined'
                else:
                    patch_frames(child)
    patch_frames(nav)
    nav['amcl']['ros__parameters']['robot_model_type'] = 'nav2_amcl::OmniMotionModel'
    controller = nav['controller_server']['ros__parameters']
    controller.update(controller_frequency=10.0, min_y_velocity_threshold=0.001,
                      odom_topic='/odom_combined')
    controller['progress_checker'].update(required_movement_radius=0.15, movement_time_allowance=15.0)
    controller['general_goal_checker'].update(xy_goal_tolerance=0.15, yaw_goal_tolerance=0.25)
    controller['FollowPath'].update(
        debug_trajectory_details=False, min_vel_x=-0.15, max_vel_x=0.20,
        min_vel_y=-0.15, max_vel_y=0.15, max_vel_theta=0.5,
        max_speed_xy=0.20, min_speed_theta=0.0,
        acc_lim_x=0.4, acc_lim_y=0.4, acc_lim_theta=0.8,
        decel_lim_x=-0.4, decel_lim_y=-0.4, decel_lim_theta=-0.8,
        vx_samples=12, vy_samples=9, vtheta_samples=16,
        trans_stopped_velocity=0.03, xy_goal_tolerance=0.15,
        critics=['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'PathAlign', 'PathDist', 'GoalDist'])
    controller['FollowPath']['ObstacleFootprint.scale'] = 1.0
    # Conservative footprint covering the V550 CAD body and all four wheels.
    footprint = '[[-0.16,-0.14],[-0.16,0.14],[0.16,0.14],[0.16,-0.14]]'
    for name in ('local_costmap', 'global_costmap'):
        costmap = nav[name][name]['ros__parameters']
        costmap.pop('robot_radius', None)
        costmap.update(footprint=footprint, footprint_padding=0.01)
        costmap['inflation_layer'].update(inflation_radius=0.35, cost_scaling_factor=4.0)
        layer = costmap.get('obstacle_layer', costmap.get('voxel_layer'))
        layer['scan'].update(obstacle_max_range=5.0, raytrace_max_range=6.0,
                             observation_persistence=0.0, expected_update_rate=0.5)
        if name == 'local_costmap':
            costmap['plugins'] = ['obstacle_layer', 'inflation_layer']
            costmap.pop('voxel_layer', None)
            costmap['obstacle_layer'] = {
                'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                'observation_sources': 'scan', 'scan': layer['scan']}
    nav['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=True, tolerance=0.25)
    nav['behavior_server']['ros__parameters'].update(
        max_rotational_vel=0.5, min_rotational_vel=0.1, rotational_acc_lim=0.8)
    nav['velocity_smoother']['ros__parameters'].update(
        max_velocity=[0.2, 0.15, 0.5], min_velocity=[-0.15, -0.15, -0.5],
        max_accel=[0.4, 0.4, 0.8], max_decel=[-0.4, -0.4, -0.8])
    (config / 'nav2_v550.yaml').write_text('# V550 mecanum: x/y holonomic DWB, metres and radians.\n' + yaml.safe_dump(nav, sort_keys=False))
    slam = yaml.safe_load((ROOT / 'src/wheeltec_robot_slam/wheeltec_slam_toolbox/config/mapper_params_online_async.yaml').read_text())
    slam['slam_toolbox']['ros__parameters'].update(
        use_sim_time=False, map_update_interval=1.0, minimum_travel_distance=0.1,
        minimum_travel_heading=0.1, max_laser_range=12.0)
    (config / 'slam_v550.yaml').write_text(yaml.safe_dump(slam, sort_keys=False))
    # Keep old known-map/voice entry points usable with the selected chassis.
    launch = ROOT / 'src/wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py'
    data = launch.read_text()
    start = data.index('    #Modify the model parameter file')
    end = data.index('    return LaunchDescription', start)
    data = data[:start] + "    param_file = LaunchConfiguration('params', default=os.path.join(\n        get_package_share_directory('v550_navigation'), 'config', 'nav2_v550.yaml'))\n\n" + data[end:]
    launch.write_text(data)
    manifest = ROOT / 'src/wheeltec_robot_nav2/package.xml'
    manifest.write_text(manifest.read_text().replace('<depend>nav2_bringup</depend>',
        '<depend>nav2_bringup</depend>\n  <exec_depend>v550_navigation</exec_depend>'))
    print('V550 configurations generated', flush=True)


def remove(path):
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)


def prune():
    assert (BACKUP / 'original_sources.tar.gz').exists(), 'Create the backup first'
    packages = []
    for name in REMOVE:
        for manifest in (ROOT / 'src' / name).rglob('package.xml'):
            packages.append(ET.parse(manifest).getroot().findtext('name'))
    # Abort before deletion if any declared retained package depends on a removed package.
    for manifest in (ROOT / 'src').rglob('package.xml'):
        tree = ET.parse(manifest).getroot()
        if tree.findtext('name') not in packages:
            for child in tree:
                assert not ('depend' in child.tag and child.text in packages), (manifest, child.text)
    removed_paths = []
    for name in REMOVE:
        removed_paths.append('src/' + name)
    for name in packages:
        removed_paths.extend(['build/' + name, 'install/' + name])
    for parent in (ROOT / 'src/wheeltec_robot_urdf/wheeltec_robot_urdf',
                   ROOT / 'install/wheeltec_robot_urdf/share/wheeltec_robot_urdf'):
        for folder, keep in (('meshes', 'V550_mec_robot_meshes'), ('urdf', 'V550_mec_robot.urdf')):
            for path in (parent / folder).iterdir():
                if path.name != keep:
                    removed_paths.append(str(path.relative_to(ROOT)))
    # Legacy vehicle tuning is archived; old launch now uses the shared V550 config.
    removed_paths.extend(['src/wheeltec_robot_nav2/param', 'install/wheeltec_nav2/share/wheeltec_nav2/param'])
    (BACKUP / 'removed.json').write_text(json.dumps({'packages': packages, 'paths': removed_paths}, indent=2))
    for path in removed_paths:
        remove(ROOT / path)
    expected = json.loads((BACKUP / 'peripheral_sha256.json').read_text())
    assert hashes() == expected, 'Peripheral driver source changed!'
    print(f'Removed {len(packages)} packages, unused vehicle assets and their build/install outputs.', flush=True)
    print('All peripheral driver files match their pre-cleanup SHA256 hashes.', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['backup', 'configure', 'prune'])
    globals()[parser.parse_args().action]()
