# IMU Sensor ROS 2 Driver (Jazzy Jalisco Compatible)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-brightgreen)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

![IMU Visualization](docs/imgs/imu_rviz.png)

A production-ready ROS 2 driver for IMU sensors with serial/UART interface, featuring native Jazzy Jalisco support and TF integration.

## 🚀 Features

- **ROS 2 Jazzy Native**: Lifecycle node implementation
- **Multi-Sensor Support**: BMI088, ICM-20948, MPU-6050
- **Advanced Data Processing**:
  - Onboard sensor fusion
  - Gravity compensation
  - Magnetometer calibration
- **TF Ready**: Automatic `base_link` → `imu_link` → `mag_link` transforms
- **Configurable**: Dynamic parameters via ROS 2 services

## 📦 Hardware Support

| Sensor Model | Interface | Tested Version |
|-------------|-----------|----------------|
| Generic IMU | UART | Protocol v2.1 |
| BMI085 | Serial | v1.4 |

## 🔧 Installation

### Prerequisites
- ROS 2 Jazzy ([install guide](https://docs.ros.org/en/jazzy/Installation.html))
- Python 3.10+
- `pyserial`:
  ```bash
  pip install pyserial

# Clone repository
git clone https://github.com/your-repo/imu_sensor_ros2.git

# Build package
cd ~/ros2_ws
colcon build --symlink-install --packages-select imu_sensor
source install/setup.bash

# Usage
ros2 launch imu_sensor imu_sensor.launch.py

# Custom Launch Example
ros2 launch imu_sensor imu_sensor.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=921600 \
  imu_id:=base_imu