#!/usr/bin/env bash

set -e

sudo apt update && sudo apt-get install -y \
  git \
  libusb-1.0-0-dev \
  libftdi1-dev \
  libuvc-dev \
  ros-${ROS_DISTRO}-sophus \
  ros-${ROS_DISTRO}-ecl-tools \
  ros-${ROS_DISTRO}-kobuki-ros-interfaces \
  ros-${ROS_DISTRO}-kobuki-velocity-smoother \
  # ros-${ROS_DISTRO}-kobuki-core \
  # ros-${ROS_DISTRO}-kobuki-ftdi \
  # ros-${ROS_DISTRO}-kobuki-firmware \
  
ROS_WS=/workspace/dev_ws

sudo rm -rf $ROS_WS/src/kobuki_ros

if [ ! -d $ROS_WS/src/kobuki_ros ]; then

  mkdir -p $ROS_WS/src

  cd $ROS_WS/src

  # tested on jazzy
  # git clone https://github.com/stonier/sophus.git -b 1.3.1
  # git clone https://github.com/stonier/ecl_tools.git -b 1.0.3

  # git clone https://github.com/stonier/ecl_core.git -b 1.2.1
  # git clone https://github.com/stonier/ecl_lite.git -b 1.2.0

  # git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git
  git clone https://github.com/kobuki-base/cmd_vel_mux.git

  git clone https://github.com/kobuki-base/kobuki_core.git -b 1.4.1
  # git clone https://github.com/kobuki-base/kobuki_ros.git -b 1.2.0

  ### REF: https://github.com/IntelligentRoboticsLabs/kobuki/blob/jazzy/thirdparty.repos
  git clone https://github.com/Juancams/ecl_core.git -b devel
  git clone https://github.com/Juancams/ecl_lite.git -b ros2-jazzy
  git clone https://github.com/Juancams/kobuki_ros.git -b jazzy-devel

fi

sudo apt-get update #&& sudo apt-get upgrade -y 

# make and install
cd $ROS_WS

sudo rosdep install -i --from-path src --rosdistro jazzy -y
# sudo rosdep install --from-paths src --ignore-src -r -y
  
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

# Install additionals and clean
sudo apt-get install  -y \
  man \
  && sudo apt-get autoremove -y \
  && sudo apt-get clean \
  && sudo rm -rf /var/lib/apt/lists/*
