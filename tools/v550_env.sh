# Source this file in each V550 terminal, or from a V550 helper.
V550_WORKSPACE="${V550_WORKSPACE:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
# Drop stale paths left in an already-open terminal after package removal.
for v550_var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
  v550_clean=()
  IFS=: read -r -a v550_entries <<< "${!v550_var}"
  for v550_entry in "${v550_entries[@]}"; do
    if [[ -n "$v550_entry" && -d "$v550_entry" ]]; then
      v550_clean+=("$v550_entry")
    fi
  done
  printf -v "$v550_var" '%s' "$(IFS=:; echo "${v550_clean[*]}")"
  export "$v550_var"
done
unset v550_var v550_clean v550_entries v550_entry
source /opt/ros/humble/setup.bash
if [[ -f "$V550_WORKSPACE/install/local_setup.bash" ]]; then
  source "$V550_WORKSPACE/install/local_setup.bash"
fi

# All nodes in the default V550 setup run on the robot, including remote-desktop
# RViz. Keep other robots on the LAN out of scan/odom/TF/cmd_vel. Set an explicit
# ROS_LOCALHOST_ONLY=0 and a dedicated shared ROS_DOMAIN_ID for multi-host ROS.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
