#!/usr/bin/env python3
"""Check variable ray counts, blind sectors, circular angles and near obstacles."""
import importlib.util
import math
from pathlib import Path

from sensor_msgs.msg import LaserScan

source = Path(__file__).resolve().parents[1] / 'src/v550_navigation/scripts/normalize_scan.py'
spec = importlib.util.spec_from_file_location('normalize_scan', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def scan(size, angle_min=0.0):
    message = LaserScan()
    message.header.frame_id = 'laser'
    message.header.stamp.sec = 123
    message.angle_min = angle_min
    message.angle_increment = 2 * math.pi / size
    message.angle_max = angle_min + (size - 1) * message.angle_increment
    message.range_min = 0.15
    message.range_max = 12.0
    message.ranges = [float('inf')] * size
    return message


# Actual M10_P frame lengths observed while driving must produce one geometry.
geometries = set()
for size in (1653, 1666, 1669, 1678, 1697):
    message = scan(size)
    message.ranges[size // 2] = 2.0
    result = module.normalize_scan(message)
    geometries.add((len(result.ranges), result.angle_min, result.angle_max, result.angle_increment))
    assert result.header == message.header
    assert len(result.ranges) == 1440
    nearest = min(range(len(result.ranges)), key=lambda i: result.ranges[i])
    assert abs(nearest - 720) <= 1, 'Rear obstacle changed direction'
    assert sum(math.isfinite(x) for x in result.ranges) == 1, 'Blind sectors were interpolated'
assert len(geometries) == 1

# Multiple returns in a bin must preserve the nearer obstacle and its intensity.
message = scan(2880)
message.ranges[100] = 3.0
message.ranges[99] = 0.8
message.intensities = [0.0] * 2880
message.intensities[99] = 7.0
message.ranges[1] = float('nan')
message.ranges[3] = 0.01
message.ranges[5] = 100.0
result = module.normalize_scan(message)
assert abs(result.ranges[50] - 0.8) < 1e-6
assert result.intensities[50] == 7.0
assert sum(math.isfinite(x) for x in result.ranges) == 1

# A scan starting at -pi still puts a forward obstacle at zero, not behind.
message = scan(360, -math.pi)
message.ranges[180] = 1.0
result = module.normalize_scan(message)
assert result.ranges[0] == 1.0
message.angle_increment /= 2
try:
    module.normalize_scan(message)
except ValueError:
    pass
else:
    raise AssertionError('Partial scan was silently treated as a full revolution')
print('PASS: fixed ray geometry, timestamps, obstacle directions, nearest returns, and blind sectors')
