FROM ros:jazzy-ros-core-noble

LABEL maintainer="Waipot Ngamsaad <waipotn@hotmail.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

#RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

# Setup ROS 2 environment
COPY ./kobuki-ros-jazzy.sh /
RUN mkdir -p $HOME/dev_ws/src && \
    bash /kobuki-ros-jazzy.sh && \
    sudo rm -f /kobuki-ros-jazzy.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/dev_ws/install/setup.bash" >> $HOME/.bashrc && \
    echo "DONE"

COPY ./kobuki_node_params.yaml /root/dev_ws/src/kobuki_ros/kobuki_node/config/

# Launch kobuki_node on container start
CMD ["bash", "-c", "source $HOME/dev_ws/install/setup.bash && ros2 launch kobuki_node kobuki_node-composed-launch.py"]
