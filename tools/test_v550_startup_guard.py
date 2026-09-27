#!/usr/bin/env python3
"""Test the installed startup guard without starting any hardware nodes."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

path = Path(get_package_share_directory('v550_navigation')) / 'launch/unknown_navigation.launch.py'
spec = importlib.util.spec_from_file_location('v550_guard_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

with tempfile.TemporaryDirectory(prefix='v550_guard_test_') as folder:
    lock = Path(folder) / 'test.lock'
    module._HARDWARE_LOCK = lock
    module._active_drivers = lambda: []
    module._acquire_hardware(None)
    # A different process must not acquire the same lock.
    code = '''import fcntl, sys
with open(sys.argv[1], 'a+') as f:
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit(0)
    sys.exit(1)
'''
    result = subprocess.run([sys.executable, '-c', code, str(lock)])
    assert result.returncode == 0, 'Concurrent launch was not blocked'
    module._hardware_lock_handle.close()
    module._hardware_lock_handle = None
    module._acquire_hardware(None)  # Leftover file after exit must not block restart.
    module._hardware_lock_handle.close()
    module._hardware_lock_handle = None
    module._active_drivers = lambda: ['wheeltec_robot_node (PID simulated)']
    try:
        module._acquire_hardware(None)
    except RuntimeError as exc:
        assert '已有底盘' in str(exc)
    else:
        raise AssertionError('Existing legacy driver was not detected')
print('PASS: duplicate launch blocked, clean restart allowed, existing legacy driver blocked')
