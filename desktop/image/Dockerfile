# FROM ros:foxy-ros-base-focal
# FROM ros:galactic-ros-base-focal
# FROM ros:humble-ros-base-jammy
FROM ros:jazzy-ros-base-noble

LABEL maintainer="Waipot Ngamsaad <waipotn@hotmail.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

#RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

# install Ubuntu packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    apt-transport-https \
    curl \
    git \
    wget \
    nano \
    sudo \
    bash-completion \
    #less \
    htop \
    tmux \
    terminator \
    xfe \
    #fluxbox \
    jwm \
    xfonts-base \
    xauth \
    x11-xkb-utils \
    xkb-data \
    dbus-x11 \
    net-tools \
    iputils-ping \
    usbutils \
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    novnc \
    #python-pip \
    python3-dev \
	python3-pip \
    python3-numpy \
    python3-scipy \
    gfortran \
    texinfo \
    libfftw3-dev \
    libconfig-dev \
    libasound2-dev \
    libgbm-dev \
    libnss3-dev \
	libssl-dev \
	libffi-dev \
	libatlas3-base \
	portaudio19-dev \
	ffmpeg \
	alsa-utils \
    libi2c-dev \
    build-essential \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install ROS packages
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

RUN apt-get update --fix-missing && apt-get upgrade --fix-missing -y
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    python3-rosdep \
    python3-colcon-common-extensions \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# init rosdep
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rm /etc/apt/apt.conf.d/docker-clean

# update pip and install some packages
RUN echo "Installing supervisor..." && \
    # curl https://bootstrap.pypa.io//get-pip.py | python3 - && \
    #curl https://bootstrap.pypa.io/pip/2.7/get-pip.py | python - && \
    pip3 install -U --no-cache-dir --break-system-packages \
    	# argcomplete \
        libconf \
        supervisor \
        supervisor_twiddler

# install nodejs
# RUN sh -c 'echo "deb https://deb.nodesource.com/node_22.x `lsb_release -cs` main" > /etc/apt/sources.list.d/nodesource.list' && \
#     curl -sSL https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add -

# RUN apt-get update && apt-get upgrade -y
# RUN apt-get install -y \
#     nodejs \
# 	&& rm -rf /var/lib/apt/lists/*

# install nodejs
RUN curl -sL https://deb.nodesource.com/setup_22.x -o nodesource_setup.sh && \
    bash nodesource_setup.sh && \
    apt install nodejs -y && \
    rm -rf nodesource_setup.sh && \
    node -v && \
    npm -v

# update tigervnc
#COPY ./app/install/tigervnc.sh /

#RUN /tigervnc.sh && sudo rm -f /tigervnc.sh

# install noVNC
#COPY ./app/install/no-vnc.sh /

#RUN /no-vnc.sh && sudo rm -f /no-vnc.sh

# install code-server
#COPY ./app/install/code-server.sh /

#RUN /code-server.sh 4.8.3 && sudo rm -f /code-server.sh

# setup user space
RUN mkdir -p {/workspace,/app}

ARG USERNAME="ubuntu"
RUN echo "Creaing user: ${USERNAME}" && \
    # useradd -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    usermod --shell /bin/bash ${USERNAME} && \
    chown -R ${USERNAME}:${USERNAME} /workspace && \
    chown -R ${USERNAME}:${USERNAME} /app && \
    ln -sfn /workspace /home/${USERNAME}/workspace && \
    ln -sfn /app /home/${USERNAME}/app && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV USER ${USERNAME}

ENV HOME /home/${USERNAME}

ENV SHELL /bin/bash

#========== USER SPACE ==========#
USER $USER

WORKDIR $HOME

# init rosdep
RUN rosdep fix-permissions && rosdep update

# colorize less
#RUN echo "export LESS='-R'" >> ~/.bash_profile && \
#    echo "export LESSOPEN='|pygmentize -g %s'" >> ~/.bash_profile

# enable bash completion
RUN echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
    echo "source ~/.bashrc" >> ~/.bash_profile 

# install ydlidar_sdk first
#RUN cd ~ && \
#    git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
#    mkdir -p YDLidar-SDK/build && \
#    cd YDLidar-SDK/build && \
#    cmake .. && \
#    sudo make install && \
#    rm -rf ~/YDLidar-SDK && \
#    echo "DONE"

# compile turtlebot2 packages from sources
#COPY ./app/install/turtlebot2.sh /

RUN mkdir -p /workspace/dev_ws/src && \
    cd /workspace/dev_ws && \
    sudo rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install && \
    echo "DONE"

# https://superuser.com/questions/555310/bash-save-history-without-exit
RUN echo "export PROMPT_COMMAND='history -a'" >> ~/.bashrc && \ 
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/dev_ws/install/setup.bash" >> ~/.bashrc && \
    # echo "export TURTLEBOT_BASE=kobuki" >> ~/.bashrc && \
    # echo "export TURTLEBOT_3D_SENSOR=asus_xtion_pro" >> ~/.bashrc && \
    # echo "export TURTLEBOT_MAP_FILE=~/workspace/maps/map.yaml" >> ~/.bashrc && \
    echo "Done"

COPY ./test_imu.rviz /home/${USERNAME}/
COPY ./view_imu.sh /home/${USERNAME}/

COPY ./index.html /usr/share/novnc
COPY ./app/install/code-server.sh /app/install/

# install code-server cert.
RUN /app/install/code-server.sh 4.98.2

COPY ./app /app
RUN sudo chown -R ${USERNAME}:${USERNAME} /app && \
    sudo usermod -aG dialout ${USERNAME} && \
    sudo usermod -aG video ${USERNAME} && \
    sudo usermod -aG i2c ${USERNAME} && \
    # sudo groupmod -g 998 i2c
    echo "\n"

# fix code-server cert.
RUN /app/install/fix-code-server-cert.sh

VOLUME /tmp/.X11-unix

ENV DISPLAY ":9"

EXPOSE 8001 6901

# clean
RUN sudo apt-get -y autoremove && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

CMD ["sudo", "-E", "/usr/local/bin/supervisord", "-c", "/app/supervisord.conf"]
