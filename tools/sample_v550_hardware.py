#!/usr/bin/env python3
"""Bounded sensor-only capture on a private ROS domain, without Nav2 or goals."""
import os
import argparse
from pathlib import Path
import signal
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--slam', action='store_true', help='Also sample online SLAM, with navigation disabled')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        exe = path.read_bytes().split(b'\0')[0].decode()
    except (OSError, UnicodeError):
        continue
    if exe.endswith(('/wheeltec_robot_node', '/lslidar_driver_node', '/yesense_node_publisher')):
        raise SystemExit('An existing chassis/lidar/IMU process is running. Refusing a second serial connection.')
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/v550_sensor_diagnostic')
with (root / 'log/v550_sensor_only.log').open('w') as log:
    process = subprocess.Popen([
        'ros2', 'launch', 'v550_navigation', 'unknown_navigation.launch.py',
        'start_slam:=' + str(args.slam).lower(), 'start_nav2:=false', 'use_rviz:=false'],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(4)
        if process.poll() is not None:
            raise SystemExit('Sensor launch exited; see log/v550_sensor_only.log')
        subprocess.run(['python3', str(root / 'tools/diagnose_v550_tf.py'), '--seconds', '15'],
                       env=env, check=True, timeout=30)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
