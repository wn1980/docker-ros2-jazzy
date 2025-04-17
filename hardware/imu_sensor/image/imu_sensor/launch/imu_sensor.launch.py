from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # IMU Sensor Node
        Node(
            package='imu_sensor',
            executable='imu_sensor_node',
            name='imu_sensor',
            output='screen',
            parameters=[{
                'port': '/dev/imu_sensor',
                'baudrate': 115200,
                'imu_id': 'imu_frame',
                'mag_id': 'mag_frame',
                'gravity': True
            }]
        ),
        
        # Static TF from module_frame to imu_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            output='screen',
            arguments=[
                '--x', '0.007',  # Forward offset (adjust as needed)
                '--y', '0.0',   # Left/right offset
                '--z', '0.0',   # Height offset
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'module_frame',
                '--child-frame-id', 'imu_frame'
            ]
        ),
        
        # Static TF from module_frame to mag_frame (if magnetometer is co-located)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mag_tf_publisher',
            output='screen',
            arguments=[
                '--x', '0.007',  # Offset from IMU if separate
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'module_frame',
                '--child-frame-id', 'mag_frame'
            ]
        ),
        
    ])