#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/dev_ws/install/setup.bash

# Check if at least a package name and launch file are provided as arguments
if [ -n "$1" ] && [ -n "$2" ]; then
  PACKAGE_NAME="$1"
  LAUNCH_FILE="$2"

  # Shift the arguments to remove the package name and launch file
  shift
  shift

  # Construct the ros2 launch command with any remaining arguments
  LAUNCH_COMMAND="ros2 launch $PACKAGE_NAME $LAUNCH_FILE $*"

  echo "Executing ROS 2 Launch: $LAUNCH_COMMAND"
  exec $LAUNCH_COMMAND
else
  # If package name and launch file are not provided, run the custom command
  if [ -n "$@" ]; then
    CUSTOM_COMMAND="$@"
    echo "Executing Custom Command: $CUSTOM_COMMAND"
    exec $CUSTOM_COMMAND
  else
    echo "Usage (ros2 launch): $0 <package_name> <launch_file> [launch_args...]"
    echo "Example (ros2 launch): $0 my_robot_bringup my_robot.launch.py use_sim:=True"
    echo "Usage (custom command): $0 <custom_command> [args...]"
    echo "Example (custom command): $0 ros2 topic list"
    exit 1
  fi
fi
