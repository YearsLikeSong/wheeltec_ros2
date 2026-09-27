#!/usr/bin/env bash
set -e
V550_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$V550_WORKSPACE/tools/v550_env.sh"
V550_MAP_NAME="${1:-v550_$(date +%Y%m%d_%H%M%S)}"
if [[ "$V550_MAP_NAME" == */* || "$V550_MAP_NAME" == -* ]]; then
  echo 'Use a map name without slashes or a leading dash.' >&2
  exit 1
fi
mkdir -p "$V550_WORKSPACE/maps"
if [[ -e "$V550_WORKSPACE/maps/$V550_MAP_NAME.yaml" ]]; then
  echo "Map already exists: $V550_MAP_NAME; choose a new name." >&2
  exit 1
fi
exec ros2 run nav2_map_server map_saver_cli -f "$V550_WORKSPACE/maps/$V550_MAP_NAME" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
