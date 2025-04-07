#!/usr/bin/env bash

set -e

sudo apt update && sudo apt-get install -y \
git

TURTLEBOT_WS=/workspace/dev_ws
sudo rm -rf $TURTLEBOT_WS/src/turtlebot2_ros2

if [ ! -d $TURTLEBOT_WS/src/turtlebot2_ros2 ]; then

  mkdir -p $TURTLEBOT_WS/src

  cd $TURTLEBOT_WS/src

  git clone https://github.com/wn1980/turtlebot2_ros2.git

fi

sudo apt-get update #&& sudo apt-get upgrade -y 

# make and install
cd $TURTLEBOT_WS

# sudo rosdep install -i --from-path src --rosdistro humble -y
sudo rosdep install --from-paths src --ignore-src -r -y
  
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

# Install additionals and clean
sudo apt-get install  -y \
  #ros-${ROS_DISTRO}-kobuki-core \
  #ros-${ROS_DISTRO}-kobuki-ftdi \
  #ros-${ROS_DISTRO}-kobuki-firmware \
  man \
  && sudo apt-get autoremove -y \
  && sudo apt-get clean \
  && sudo rm -rf /var/lib/apt/lists/*
