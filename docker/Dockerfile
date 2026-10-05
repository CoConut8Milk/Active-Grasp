# ROS2 Humble + Gazebo Classic 11 environment for this project.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    python3-pip python3-numpy python3-scipy python3-yaml \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy==1.26.4 \
    && pip3 install --no-cache-dir torch==2.0.1 --index-url https://download.pytorch.org/whl/cpu

WORKDIR /workspace

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

