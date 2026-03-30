#!/usr/bin/env bash
# Kill leftover Gazebo processes (fixes "Address already in use" on gzserver)
killall -9 gzserver gzclient 2>/dev/null || true
