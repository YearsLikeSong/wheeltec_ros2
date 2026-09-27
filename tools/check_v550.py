#!/usr/bin/env python3
"""Static validation of V550 packaging and unchanged peripheral drivers."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

root = Path(__file__).resolve().parents[1]
backup = root / 'backups/v550_before_cleanup'
removed = json.loads((backup / 'removed.json').read_text())['packages']
before = json.loads((backup / 'peripheral_sha256.json').read_text())
for name, digest in before.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
packages = {}
for manifest in (root / 'src').rglob('package.xml'):
    tree = ET.parse(manifest).getroot()
    name = tree.findtext('name')
    assert name not in removed, name
    packages[name] = manifest
    for dependency in tree:
        assert not ('depend' in dependency.tag and dependency.text in removed), (name, dependency.text)
for name in removed:
    assert not (root / 'install' / name).exists(), name
    assert not (root / 'build' / name).exists(), name
for name in ('v550_navigation', 'wheeltec_nav2', 'turn_on_wheeltec_robot',
             'wheeltec_robot_urdf', 'lslidar_driver', 'astra_camera', 'usb_cam',
             'yesense_std_ros2', 'slam_toolbox', 'robot_localization',
             'imu_filter_madgwick', 'nav2_bringup'):
    assert Path(get_package_share_directory(name)).is_dir(), name
for folder in ('src/v550_navigation', 'src/turn_on_wheeltec_robot/launch'):
    for path in (root / folder).rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
model = root / 'src/wheeltec_robot_urdf/wheeltec_robot_urdf'
tree = ET.parse(model / 'urdf/V550_mec_robot.urdf').getroot()
children = [joint.find('child').get('link') for joint in tree.findall('joint')]
assert len(children) == len(set(children))
for mesh in tree.findall('.//mesh'):
    assert (model / mesh.get('filename').split('wheeltec_robot_urdf/', 1)[1]).is_file()
assert [p.name for p in (model / 'urdf').glob('*.urdf')] == ['V550_mec_robot.urdf']
nav_path = root / 'src/v550_navigation/config/nav2_v550.yaml'
nav = yaml.safe_load(nav_path.read_text())
controller = nav['controller_server']['ros__parameters']
assert controller['min_y_velocity_threshold'] < controller['FollowPath']['max_vel_y']
assert controller['FollowPath']['min_vel_y'] < 0 < controller['FollowPath']['max_vel_y']
assert controller['FollowPath']['vy_samples'] > 1
assert controller['FollowPath']['acc_lim_y'] > 0
assert controller['FollowPath']['decel_lim_y'] < 0
alias = Path(get_package_share_directory('wheeltec_nav2')) / 'param/wheeltec_params/param_mini_mec.yaml'
assert alias.read_bytes() == nav_path.read_bytes(), 'Voice compatibility parameters are stale'
assert (root / 'src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp').is_file()
print(f'PASS: {len(packages)} source packages; {len(removed)} removed packages absent from source/build/install; {len(before)} peripheral files unchanged; V550 model/meshes, lateral control and voice compatibility valid.')
