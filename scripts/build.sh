#!/usr/bin/env bash
# Build all packages.
set -euo pipefail

cd "$(dirname "$0")/.."

# ROS 2 Humble 的 setup.bash 会引用未定义变量 AMENT_TRACE_SETUP_FILES，
# 与 set -u 冲突；source 期间临时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install
echo
echo "Build finished. Source the workspace with:"
echo "  source install/setup.bash"
