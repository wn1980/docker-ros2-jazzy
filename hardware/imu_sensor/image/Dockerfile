FROM ros:jazzy-ros-core-noble

LABEL maintainer="Waipot Ngamsaad <waipotn@hotmail.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

#RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

# Setup ROS 2 environment
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-robot-localization \
    ros-dev-tools \
    python3-pip

RUN rosdep init && rosdep update && \
    pip install pyserial --break-system-package

RUN mkdir -p /workspace/dev_ws/src
COPY ./imu_sensor /workspace/dev_ws/src
RUN cd /workspace/dev_ws/ && \
    rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc && \
    echo "source /workspace/dev_ws/install/setup.bash" >> $HOME/.bashrc && \
    echo "DONE"

# Copy the entrypoint script into the container
RUN rm -f /ros_entrypoint.sh
COPY bin/entrypoint.sh /
COPY app /app

# Set the entrypoint to the bash script
ENTRYPOINT ["/entrypoint.sh"]

# Launch node on container start
CMD ["ros2 launch imu_sensor imu_sensor.launch.py"]
