#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_root}"

: "${ROS_DISTRO:=humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${workspace_root}/install/setup.bash"
  set -u
fi

# Sandbox-friendly default: keep ROS logs inside the workspace.
export ROS_HOME="${ROS_HOME:-${workspace_root}/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
mkdir -p "${ROS_LOG_DIR}"

exec ros2 launch nao_description display.launch.py "$@"
