#!/usr/bin/env bash
# One-shot Ubuntu 22.04 / WSL2 environment setup for ROS2 Humble + Gazebo Classic.
# Run inside an Ubuntu 22.04 terminal:
#   bash scripts/setup_wsl2.sh
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  curl gnupg2 lsb-release software-properties-common

if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-ros-base \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  python3-matplotlib

sudo rosdep init || true
rosdep update

python3 -m pip install --user --upgrade pip
python3 -m pip install --user numpy==1.26.4
python3 -m pip install --user torch==2.0.1 --index-url https://download.pytorch.org/whl/cpu

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo
echo "Setup complete. Open a new terminal, then run: bash scripts/build.sh"

