FROM ros:jazzy-ros-core-noble

LABEL maintainer="Waipot Ngamsaad <waipotn@hotmail.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

#RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

# Install ROS 2 ${ROS_DISTRO} rosbridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-server \
    # python3-rosbridge-library \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Launch rosbridge on container start
CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
