#!/usr/bin/env bash
# active-grasp 一键环境安装脚本
# 必须在 Ubuntu 22.04 (jammy) 里运行——本项目依赖 ROS2 Humble，不支持
# Ubuntu 24.04 / ROS2 Jazzy。原生 Ubuntu 与 WSL2 通用。
#
#   bash scripts/setup_wsl2.sh
#
# 全程约 20–60 分钟（取决于网络与磁盘速度），无需独立显卡。
set -euo pipefail

step() { echo; echo "==> $1"; }

# ---------------------------------------------------------------
# 国内网络加速：设置 PIP_INDEX_URL 可让 pip 走清华镜像；
# rosdep 与 PyTorch 下载失败时，脚本会自动改用国内镜像重试。
# ---------------------------------------------------------------
# export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"

step "0/5 基础工具"
sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release software-properties-common

step "1/5 添加 ROS2 软件源"
CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
if [ "$CODENAME" != "jammy" ]; then
  echo "错误：检测到 Ubuntu $CODENAME，本项目只支持 22.04 (jammy)。" >&2
  echo "请重装 Ubuntu 22.04，或在 WSL2 中安装 Ubuntu-22.04 发行版。" >&2
  exit 1
fi
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update

step "2/5 安装 ROS2 Humble + Gazebo Classic（这一步最久）"
sudo apt-get install -y \
  ros-humble-desktop \
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
  ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  python3-matplotlib

step "3/5 初始化 rosdep（国内网络自动改用清华镜像）"
sudo rosdep init >/dev/null 2>&1 || true
if ! rosdep update >/dev/null 2>&1; then
  echo "  默认 rosdep 源不可用（国内网络常见），改用清华镜像重试..."
  sudo mkdir -p /etc/ros/rosdep/sources.list.d
  if [ ! -s /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo curl -fsSL -o /etc/ros/rosdep/sources.list.d/20-default.list \
      https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list || true
  fi
  sudo sed -i 's|raw.githubusercontent.com/ros/rosdistro/master|mirrors.tuna.tsinghua.edu.cn/rosdistro|g' \
    /etc/ros/rosdep/sources.list.d/20-default.list || true
  export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
  grep -q "ROSDISTRO_INDEX_URL" ~/.bashrc || \
    echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
  rosdep update || echo "  提示：rosdep 更新失败（不影响本项目编译运行，可稍后重试）"
fi

step "4/5 安装 Python 依赖"
# numpy 固定 1.26.4：满足 PyTorch，同时保持 ROS2 Humble 的 NumPy 1.x C-ABI。
# 不要升级到 numpy 2.x，会破坏 cv_bridge 等二进制扩展。
python3 -m pip install --user --upgrade pip || echo "  提示：pip 升级失败，继续使用系统 pip"
if ! python3 -m pip install --user numpy==1.26.4; then
  echo "  PyPI 默认源不可用，改用清华镜像重试..."
  python3 -m pip install --user numpy==1.26.4 \
    -i https://pypi.tuna.tsinghua.edu.cn/simple || echo "  提示：numpy 安装失败，可稍后手动重试"
fi
if ! python3 -m pip install --user torch==2.0.1 --index-url "$TORCH_INDEX"; then
  echo "  官方 PyTorch 源不可用，改用阿里云镜像重试..."
  python3 -m pip install --user "torch==2.0.1+cpu" \
    -f https://mirrors.aliyun.com/pytorch-wheels/cpu/ \
    -i "${PIP_INDEX_URL:-https://pypi.org/simple}" || \
    echo "  提示：PyTorch 安装失败，可稍后手动重试（训练时才需要）"
fi

step "5/5 写入环境变量"
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo
echo "安装完成。接下来按顺序执行："
echo "  1) 重开一个终端（让环境变量生效）"
echo "  2) cd <项目目录> && bash scripts/check_env.sh   # 环境自检"
echo "  3) bash scripts/build.sh                        # 编译"
