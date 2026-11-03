#!/bin/bash
# 安装 ROS1 Noetic 及 FAST-LIVO2 依赖（需 sudo）。
# cv_bridge 不用 apt 装：apt 版链接 OpenCV 4.2，而系统里是 JetPack 的 OpenCV 4.5，改在工作空间里从源码编译
set -e
[ "$(id -u)" = 0 ] || { echo "用法: sudo bash $0"; exit 1; }

PROXY=${PROXY:-http://127.0.0.1:17897}
export http_proxy=$PROXY https_proxy=$PROXY
APT_PROXY=(-o Acquire::http::Proxy=$PROXY -o Acquire::https::Proxy=$PROXY)

KEY=/usr/share/keyrings/ros-archive-keyring.gpg
wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor --yes -o $KEY
echo "deb [arch=arm64 signed-by=$KEY] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list

# 系统里有两个无效的第三方源（librealsense 缺公钥、pointclouds.org 已失效），update 会报错但不影响其余源
apt-get "${APT_PROXY[@]}" update || true
apt-get "${APT_PROXY[@]}" install -y \
    ros-noetic-ros-base ros-noetic-pcl-ros ros-noetic-image-transport \
    ros-noetic-tf ros-noetic-eigen-conversions \
    python3-catkin-tools libeigen3-dev libboost-all-dev
echo "完成：source /opt/ros/noetic/setup.bash"
