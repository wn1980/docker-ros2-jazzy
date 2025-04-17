#!/usr/bin/env bash

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="imu_sensor"' >/etc/udev/rules.d/imu_sensor.rules

service udev reload
sleep 1
service udev restart