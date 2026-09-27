#!/usr/bin/env bash
set -e
V550_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$V550_WORKSPACE"
source "$V550_WORKSPACE/tools/v550_env.sh"
# Reuse the existing Nav2 and driver installation; only changed/new packages.
exec colcon build --executor sequential --packages-select \
  turn_on_wheeltec_robot wheeltec_robot_urdf v550_navigation wheeltec_nav2 simple_follower_ros2 \
  --allow-overriding turn_on_wheeltec_robot --cmake-args -DBUILD_TESTING=OFF "$@"
