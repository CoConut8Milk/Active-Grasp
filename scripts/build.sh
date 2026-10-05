#!/usr/bin/env bash
# Build all packages.
set -euo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo
echo "Build finished. Source the workspace with:"
echo "  source install/setup.bash"

