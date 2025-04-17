#!/usr/bin/env python3
# Copyright 2020 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
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

import time
import serial
import struct
import sys

class Queue:
    def __init__(self, capacity=1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

class Sensor:
    def __init__(self, port='/dev/imu_sensor', baudrate=115200):
        self.G = 9.8  # EARTH_GRAVITY
        self.device_port = port
        self.baudrate = baudrate
        
        # Data holders
        self.Gyro = [0, 0, 0]
        self.Accel = [0, 0, 0]
        self.Quat = [0, 0, 0, 0]
        self.Mag = [0, 0, 0]
        self.firmware_version = [0, 0, 0]
        self.hardware_version = [0, 0, 0]
        
        self.serialIDLE_flag = 0
        self.Circleloop = Queue(capacity=1024*8)
        
        # Initialize serial communication
        try:
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=10)
            print("Opening Sensor")
            try:
                if self.serial.in_waiting:
                    self.serial.read_all()
            except:
                print("Opening Sensor Try Failed")
                pass
        except:
            print(f"Cannot open Serial {self.device_port}")
            self.serial.close()
            sys.exit(0)
            
        print("Sensor Open Succeeded")
        self.get_version()
        self.get_sn()
        time.sleep(0.1)

    def crc_1byte(self, data):
        crc_1byte = 0
        for i in range(0, 8):
            if (crc_1byte ^ data) & 0x01:
                crc_1byte ^= 0x18
                crc_1byte >>= 1
                crc_1byte |= 0x80
            else:
                crc_1byte >>= 1
            data >>= 1
        return crc_1byte
        
    def crc_byte(self, data, length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret ^ data[i])
        return ret

    def process_data(self):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading) != 0:
                for i in range(0, len(reading)):
                    data = int.from_bytes(reading[i:i+1], byteorder='big')
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        print("Circleloop.enqueue Failed")

        if not self.Circleloop.is_empty():
            data = self.Circleloop.get_front()
            if data == 0x5a:
                length = self.Circleloop.get_front_second()
                if length > 1:
                    if self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                        databuf = []
                        for i in range(length):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()
                        
                        if (databuf[length-1]) == self.crc_byte(databuf, length-1):
                            pass
                            
                        if databuf[3] == 0xf2:
                            self.hardware_version = databuf[4:7]
                            self.firmware_version = databuf[7:10]
                            version_string = (f"Sensor Hardware Ver {self.hardware_version[0]}.{self.hardware_version[1]}.{self.hardware_version[2]}, "
                                           f"Firmware Ver {self.firmware_version[0]}.{self.firmware_version[1]}.{self.firmware_version[2]}")
                            print(version_string)
                            
                        elif databuf[3] == 0x18 or databuf[3] == 0x1c:
                            for i in range(3):
                                x = databuf[i*4+4:i*4+8]
                                self.Gyro[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                            for i in range(3):
                                x = databuf[i*4+16:i*4+20]
                                self.Accel[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                            for i in range(4):
                                x = databuf[i*4+28:i*4+32]
                                self.Quat[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                                
                        elif databuf[3] == 0x1A:
                            for i in range(3):
                                x = databuf[i*4+4:i*4+8]
                                self.Mag[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                                
                        elif databuf[3] == 0xf4:
                            sn_string = "Sensor SN:"
                            for i in range(4, 16):
                                sn_string = f"{sn_string}{databuf[i]:02x}"
                            print(sn_string)
                            
                        else:
                            print(f"Invalid Index {databuf[3]}")
                            pass
            else:
                self.Circleloop.dequeue()

    def get_version(self):
        output = bytes([0x5a, 0x06, 0x01, 0xf1, 0x00, 0xd7])
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            print("Get Version Command Send Failed")
        self.serialIDLE_flag = 0

    def get_sn(self):
        output = bytes([0x5a, 0x06, 0x01, 0xf3, 0x00, 0x46])
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            print("Get SN Command Send Failed")
        self.serialIDLE_flag = 0

    def request_imu_data(self, gravity=True):
        if gravity:
            output = bytes([0x5a, 0x06, 0x01, 0x17, 0x00, 0xff])
        else:
            output = bytes([0x5a, 0x06, 0x01, 0x1b, 0x00, 0xff])
            
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            print("IMU Command Send Failed")
        self.serialIDLE_flag = 0

    def request_mag_data(self):
        output = bytes([0x5a, 0x06, 0x01, 0x19, 0x00, 0xff])
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            print("Mag Command Send Failed")
        self.serialIDLE_flag = 0

    def close(self):
        self.serial.close()


def main():
    # Initialize the sensor with your specific port and baudrate
    sensor = Sensor(port='/dev/imu_sensor', baudrate=115200)
    
    try:
        print("Testing IMU sensor communication...")
        print("Press Ctrl+C to stop")
        
        while True:
            # Process incoming data
            sensor.process_data()
            
            # Request and display IMU data
            sensor.request_imu_data(gravity=True)
            print("\nIMU Data:")
            print(f"Gyro: X={sensor.Gyro[0]:.3f}, Y={sensor.Gyro[1]:.3f}, Z={sensor.Gyro[2]:.3f} rad/s")
            print(f"Accel: X={sensor.Accel[0]:.3f}, Y={sensor.Accel[1]:.3f}, Z={sensor.Accel[2]:.3f} g")
            print(f"Quat: W={sensor.Quat[0]:.3f}, X={sensor.Quat[1]:.3f}, Y={sensor.Quat[2]:.3f}, Z={sensor.Quat[3]:.3f}")
            
            # Request and display magnetometer data
            sensor.request_mag_data()
            print("\nMagnetometer Data:")
            print(f"Mag: X={sensor.Mag[0]:.3f}, Y={sensor.Mag[1]:.3f}, Z={sensor.Mag[2]:.3f} uT")
            
            # Add some delay to not overwhelm the output
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        sensor.close()
        print("Sensor connection closed")

if __name__ == '__main__':
    main()