#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from imu_sensor import Sensor

class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/imu_sensor')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_id', 'imu_frame')
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('imu_freq', 100.0)
        self.declare_parameter('mag_id', 'mag_frame')
        self.declare_parameter('mag_topic', 'imu/mag')
        self.declare_parameter('mag_freq', 70.0)
        self.declare_parameter('gravity', True)
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.imu_id = self.get_parameter('imu_id').value
        imu_topic = self.get_parameter('imu_topic').value
        imu_freq = self.get_parameter('imu_freq').value
        self.mag_id = self.get_parameter('mag_id').value
        mag_topic = self.get_parameter('mag_topic').value
        mag_freq = self.get_parameter('mag_freq').value
        self.gravity = self.get_parameter('gravity').value
        
        # Initialize sensor driver
        self.sensor = Sensor(port=port, baudrate=baudrate)
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.mag_pub = self.create_publisher(MagneticField, mag_topic, 10)
        
        # Create timers
        self.communication_timer = self.create_timer(0.001, self.communication_callback)
        self.imu_timer = self.create_timer(1.0/imu_freq, self.imu_callback)
        self.mag_timer = self.create_timer(1.0/mag_freq, self.mag_callback)
        
    def communication_callback(self):
        self.sensor.process_data()
        
    def imu_callback(self):
        self.sensor.request_imu_data(self.gravity)
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_id
        
        msg.angular_velocity.x = float(self.sensor.Gyro[0])
        msg.angular_velocity.y = float(self.sensor.Gyro[1])
        msg.angular_velocity.z = float(self.sensor.Gyro[2])
        
        msg.linear_acceleration.x = float(self.sensor.Accel[0] * self.sensor.G)
        msg.linear_acceleration.y = float(self.sensor.Accel[1] * self.sensor.G)
        msg.linear_acceleration.z = float(self.sensor.Accel[2] * self.sensor.G)
        
        msg.orientation.w = float(self.sensor.Quat[0])
        msg.orientation.x = float(self.sensor.Quat[1])
        msg.orientation.y = float(self.sensor.Quat[2])
        msg.orientation.z = float(self.sensor.Quat[3])
        
        self.imu_pub.publish(msg)
        
    def mag_callback(self):
        self.sensor.request_mag_data()
        
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mag_id
        
        msg.magnetic_field.x = float(self.sensor.Mag[0])
        msg.magnetic_field.y = float(self.sensor.Mag[1])
        msg.magnetic_field.z = float(self.sensor.Mag[2])
        
        self.mag_pub.publish(msg)
        
    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()