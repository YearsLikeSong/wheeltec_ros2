#!/usr/bin/env bash
# One command, regardless of the caller's current directory.
set -e
V550_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$V550_WORKSPACE/tools/v550_env.sh"
if [[ ! -f "$V550_WORKSPACE/install/v550_navigation/share/v550_navigation/package.xml" ]]; then
  echo 'Build first: bash tools/build_v550.sh' >&2
  exit 1
fi
exec ros2 launch v550_navigation unknown_navigation.launch.py "$@"
