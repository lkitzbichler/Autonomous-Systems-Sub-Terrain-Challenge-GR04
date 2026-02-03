#!/usr/bin/env bash
set -euo pipefail

# Run from repo root
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${REPO_ROOT}/ros2_ws"
SIM_SRC="${REPO_ROOT}/simulation"
SIM_DST="${ROS_WS}/install/simulation/lib/simulation"

echo "[0/4] Updating apt metadata and installing system deps..."
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-octomap-server \
  ros-jazzy-pcl-ros \
  ros-jazzy-depth-image-proc \
  libgflags-dev \
  libompl-dev

echo "[1/4] Building colcon workspace..."
cd "${ROS_WS}"
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "Build OK."

# echo "[2/4] Ensuring destination exists..."
# mkdir -p "${SIM_DST}"
#
# echo "[3/4] Copy simulation runtime files if missing..."
# shopt -s nullglob dotglob
# for item in "${SIM_SRC}"/*; do
#   base="$(basename "${item}")"
#   src="${item}"
#   dst="${SIM_DST}/${base}"
#
#   if [[ -e "${dst}" ]]; then
#     echo "  - Skipping (exists): ${base}"
#   else
#     echo "  - Copying: ${base}"
#     cp -a "${src}" "${dst}"
#   fi
# done
# shopt -u nullglob dotglob
#
# echo "[4/4] Ensuring executable bit on Simulation.x86_64..."
# BIN="${SIM_DST}/Simulation.x86_64"
# if [[ -f "${BIN}" ]]; then
#   if [[ -x "${BIN}" ]]; then
#     echo "  - Already executable: Simulation.x86_64"
#   else
#     echo "  - Setting executable: Simulation.x86_64"
#     sudo chmod +x "${BIN}"
#   fi
# else
#   echo "WARN: ${BIN} not found (did copy succeed?)"
# fi

echo "Done."
