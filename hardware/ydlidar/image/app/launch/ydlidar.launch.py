#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    
     # Declare the argument for the YAML filename
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join('/app', 'params', 'X4.yaml'),
        description='FPath to the ROS2 parameters file to use.'
    )

    # Get the value of the parameter filename argument
    parameter_file = LaunchConfiguration('params_file')

    # Declare arguments for the static transform publisher
    declare_x_arg = DeclareLaunchArgument('x', default_value='0', description='X translation')
    declare_y_arg = DeclareLaunchArgument('y', default_value='0', description='Y translation')
    declare_z_arg = DeclareLaunchArgument('z', default_value='0', description='Z translation')
    declare_qx_arg = DeclareLaunchArgument('qx', default_value='0', description='x component of the quaternion')
    declare_qy_arg = DeclareLaunchArgument('qy', default_value='0', description='y component of the quaternion')
    declare_qz_arg = DeclareLaunchArgument('qz', default_value='0', description='z component of the quaternion')
    declare_qw_arg = DeclareLaunchArgument('qw', default_value='1', description='w component of the quaternion')
    declare_parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='base_link', description='Parent frame name')
    declare_child_frame_arg = DeclareLaunchArgument('child_frame', default_value='laser_frame', description='Child frame name')

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # arguments=['0', '0', '0.1','0', '0', '0', '1','base_link','laser_frame'],
        arguments=[
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('z'),
            LaunchConfiguration('qx'),
            LaunchConfiguration('qy'),
            LaunchConfiguration('qz'),
            LaunchConfiguration('qw'),
            LaunchConfiguration('parent_frame'),
            LaunchConfiguration('child_frame')
        ],
    )

    return LaunchDescription([
        declare_params,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_qx_arg,
        declare_qy_arg,
        declare_qz_arg,
        declare_qw_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        driver_node,
        tf2_node,
    ])
