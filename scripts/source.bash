#!/usr/bin/env bash
# Usage:
#   source scripts/source.bash

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

: "${ROS_DISTRO:=humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${workspace_root}/install/setup.bash"
fi
