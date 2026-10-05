#!/usr/bin/env bash
# Headless fast-physics DDQN training.
set -euo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
exec ros2 launch ag_agent train.launch.py

