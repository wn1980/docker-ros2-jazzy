FROM ros:jazzy-ros-core-noble

LABEL maintainer="Waipot Ngamsaad <waipotn@hotmail.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

#RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

# Setup ROS 2 environment
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-openni2-camera \
    ros-dev-tools

RUN rosdep init && rosdep update

RUN mkdir -p /workspace/dev_ws/src && \
    cd /workspace/dev_ws/ && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc && \
    echo "source /workspace/dev_ws/install/setup.bash" >> $HOME/.bashrc && \
    echo "DONE"

# Copy the entrypoint script into the container
RUN rm -f /ros_entrypoint.sh
COPY bin/entrypoint.sh /

# Set the entrypoint to the bash script
ENTRYPOINT ["/entrypoint.sh"]

# Launch node on container start
CMD ["ros2 launch openni2_camera camera_with_cloud.launch.py"]
