#!/usr/bin/env bash
set -eo pipefail

# ----- Config -----
ROS_DISTRO="${ROS_DISTRO:-foxy}"
ROS_WS="${ROS_WS:-/ros2_ws}"
SHARED_ROS2="${SHARED_ROS2:-/root/shared/ros2}"   # mount this if you use it
ROS_DOMAIN_ID_FILE="${ROS_DOMAIN_ID_FILE:-$SHARED_ROS2/ros_domain_id.txt}"

# ----- GUI runtime dir (optional, only if you use GUI) -----
if [[ -n "${DISPLAY:-}" ]]; then
  export XDG_RUNTIME_DIR="/tmp/runtime-$(id -un)"
  mkdir -p "$XDG_RUNTIME_DIR"
  chmod 700 "$XDG_RUNTIME_DIR"
fi

# ----- ROS env -----
# Source ROS distro environment
if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/install/setup.bash"
else
  echo "ERROR: ROS $ROS_DISTRO setup not found under /opt/ros/$ROS_DISTRO" >&2
  ls -la "/opt/ros/$ROS_DISTRO" || true
  exit 1
fi

export PYTHONPATH="/deep_sort_pytorch:$PYTHONPATH"
export PYTHONPATH="/deep_sort_pytorch/deep_sort:$PYTHONPATH"


# Source workspace (only if already built)
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_WS/install/setup.bash"
fi

# ----- ROS_DOMAIN_ID handling (no dotfile edits) -----
# If a shared file is used, read it; otherwise honor env if provided.
if [[ -f "$ROS_DOMAIN_ID_FILE" ]]; then
  export ROS_DOMAIN_ID="$(cat "$ROS_DOMAIN_ID_FILE" | tr -d '[:space:]')"
elif [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  : # keep the provided env value
else
  export ROS_DOMAIN_ID=0
fi

# Validate domain id range [0,232]
if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232 )); then
  echo "WARNING: ROS_DOMAIN_ID='$ROS_DOMAIN_ID' out of range [0,232]; defaulting to 0"
  export ROS_DOMAIN_ID=0
fi

# ----- Hand off to the requested command -----
exec "$@"


#colcon build
#. install/setup.bash