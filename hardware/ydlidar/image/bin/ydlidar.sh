#!/usr/bin/env bash

set -e

apt update && apt install -y \
  ros-${ROS_DISTRO}-tf2-ros \
  build-essential \
  git

DEV_WS=/opt/dev_ws

if [ ! -d $DEV_WS/src/ydlidar_ros2_driver ]; then

  mkdir -p $DEV_WS/src

  cd $DEV_WS/src

  git clone https://github.com/wn1980/ydlidar_ros2_driver.git

  # install ydlidar_sdk first
  cd ~
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git 
  mkdir -p YDLidar-SDK/build
  cd YDLidar-SDK/build
  cmake ..
  make install
  rm -rf ~/YDLidar-SDK

fi

apt update && apt install ros-dev-tools -y

rosdep init && rosdep update

# make and install
cd $DEV_WS

# sudo rosdep install -i --from-path src --rosdistro jazzy -y
rosdep install --from-paths src --ignore-src -r -y
  
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

# Install additionals and clean
apt install -y \
  man \
  && apt autoremove -y \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*
