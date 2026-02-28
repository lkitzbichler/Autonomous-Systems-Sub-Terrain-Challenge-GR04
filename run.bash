#!/usr/bin/env bash
set -eo pipefail  # bewusst ohne -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${REPO_ROOT}/ros2_ws"

# Source ROS 2 Jazzy base environment first.
set +u
if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  source "/opt/ros/jazzy/setup.bash"
else
  echo "ERROR: /opt/ros/jazzy/setup.bash not found. Please install ROS 2 Jazzy."
  exit 1
fi

cd "${ROS_WS}"

# colcon setup scripts mögen kein "nounset"
if [[ -f "install/local_setup.bash" ]]; then
  source "install/local_setup.bash"
elif [[ -f "install/setup.bash" ]]; then
  source "install/setup.bash"
elif [[ -f "install/local_setup.sh" ]]; then
  source "install/local_setup.sh"
elif [[ -f "install/setup.sh" ]]; then
  source "install/setup.sh"
else
  echo "ERROR: No install setup script found. Did you run colcon build?"
  exit 1
fi
set -u 2>/dev/null || true

exec ros2 launch simulation simulation.launch.py
