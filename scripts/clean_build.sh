#!/usr/bin/env bash
# Clean build to fix stale AMENT_PREFIX_PATH (nao_meshes, my_robot_pkg)
set -e
cd "$(dirname "$0")/.."
rm -rf build install log
# Unset stale overlay paths before sourcing base
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install "$@"
echo "Done. Run: source install/setup.bash"
