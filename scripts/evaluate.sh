#!/usr/bin/env bash
# Evaluate a trained checkpoint across 2/4/6/8-object scenes.
set -euo pipefail

cd "$(dirname "$0")/.."

# ROS 2 Humble 的 setup.bash 与 set -u 冲突，source 期间临时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
set -u

CHECKPOINT="${1:-checkpoints/agent_final.pth}"
exec ros2 launch ag_agent evaluate.launch.py checkpoint:="$CHECKPOINT"
