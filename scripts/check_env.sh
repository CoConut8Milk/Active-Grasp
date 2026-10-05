#!/usr/bin/env bash
# 环境自检：逐项验证 active-grasp 的依赖是否就绪。
# 在项目根目录运行：bash scripts/check_env.sh
# 全部 [OK] 后即可执行 bash scripts/build.sh

fails=0
ok()  { echo "  [OK]   $1"; }
bad() { echo "  [FAIL] $1"; fails=$((fails+1)); }

echo "== 操作系统 =="
if grep -q jammy /etc/os-release 2>/dev/null; then
  ok "Ubuntu 22.04 (jammy)"
else
  bad "Ubuntu 22.04 (jammy) —— 当前是 $(. /etc/os-release && echo "$PRETTY_NAME")"
fi

echo "== ROS2 Humble =="
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  ok "ROS2 Humble 已安装"
  command -v ros2 >/dev/null 2>&1 && ok "ros2 命令可用" || bad "ros2 命令不可用"
else
  bad "ROS2 Humble 未安装"
fi

for pkg in \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators; do
  dpkg -s "$pkg" >/dev/null 2>&1 && ok "$pkg" || bad "$pkg 未安装"
done

echo "== Gazebo Classic =="
if command -v gazebo >/dev/null 2>&1 && gazebo --version 2>&1 | grep -q "version 11"; then
  ok "Gazebo 11 (classic)"
else
  bad "Gazebo 11 不可用"
fi

echo "== Python 依赖 =="
if python3 -c "import numpy as n; assert n.__version__.startswith('1.26')" >/dev/null 2>&1; then
  ok "numpy 1.26.x"
else
  bad "numpy 不是 1.26.x（当前 $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo 未安装)）"
fi
python3 -c "import scipy" >/dev/null 2>&1 && ok "scipy" || bad "scipy 未安装"
python3 -c "import torch" >/dev/null 2>&1 && ok "torch" || bad "torch 未安装"

echo "== 编译产物 =="
if [ -d install/ag_bringup ] && [ -d install/ag_agent ]; then
  ok "工作区已编译（install/ 存在）"
else
  echo "  [--]  尚未编译（首次运行前出现这一条是正常的）"
fi

echo
if [ "$fails" -eq 0 ]; then
  echo "全部通过 ✔  可以执行：bash scripts/build.sh"
else
  echo "$fails 项未通过 ✘  请对照 docs/06_常见问题排查.md 处理"
fi
exit "$fails"
