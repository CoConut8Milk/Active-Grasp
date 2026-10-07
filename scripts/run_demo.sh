#!/usr/bin/env bash
# Realtime GUI demo with the greedy baseline.
set -euo pipefail

cd "$(dirname "$0")/.."

# ROS 2 Humble 的 setup.bash 与 set -u 冲突，source 期间临时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
set -u

exec ros2 launch ag_agent demo.launch.py
