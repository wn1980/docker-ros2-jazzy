#!/usr/bin/env bash

set -e

sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-tf2-ros \
  build-essential \
  git

DEV_WS=/workspace/dev_ws
sudo rm -rf $DEV_WS/src/ydlidar_ros2_driver
sudo rm -rf ~/YDLidar-SDK

if [ ! -d $DEV_WS/src/ydlidar_ros2_driver ]; then

  mkdir -p $DEV_WS/src

  cd $DEV_WS/src

  git clone https://github.com/wn1980/ydlidar_ros2_driver.git

  # install ydlidar_sdk first
  cd ~
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git -b V1.2.7
  mkdir -p YDLidar-SDK/build
  cd YDLidar-SDK/build
  cmake ..
  sudo make install
  rm -rf ~/YDLidar-SDK

fi

sudo apt update && sudo apt install ros-dev-tools -y
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init && rosdep update

# make and install
cd $DEV_WS

sudo rosdep install -i --from-path src --rosdistro jazzy -y
# sudo rosdep install --from-paths src --ignore-src -r -y
  
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

# Install additionals and clean
sudo apt install -y \
  man \
  && sudo apt autoremove -y \
  && sudo apt clean \
  && sudo rm -rf /var/lib/apt/lists/*
