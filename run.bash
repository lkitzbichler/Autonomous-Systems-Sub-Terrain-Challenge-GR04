#!/usr/bin/env bash
set -eo pipefail  # bewusst ohne -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${REPO_ROOT}/ros2_ws"

cd "${ROS_WS}"

# colcon setup scripts mögen kein "nounset"
set +u
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
