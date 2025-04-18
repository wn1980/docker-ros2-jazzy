# ekf.launch.py
# -------------
# Launch file for running the robot_localization EKF node for IMU/Odometry fusion
# on a 2D mobile robot.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch argument for the parameter file.  This allows the user
    # to specify a different parameter file at runtime.
    declare_params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(
            '/app', 
            'params',
            'ekf.yaml'  # Default parameter file name
        ),
        description='Path to the EKF parameter file (YAML format).'
    )

    # Get the value of the parameter file argument.
    params_file = LaunchConfiguration('params_file')

    # Launch the EKF node.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[params_file], # Pass the parameter file path to the node.
        output='screen',  #  Output to the console
    )

    # Create the launch description and add the actions.
    ld = LaunchDescription()
    ld.add_action(declare_params_file_arg)
    ld.add_action(ekf_node)
    return ld
