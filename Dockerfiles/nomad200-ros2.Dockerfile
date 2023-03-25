FROM ubuntu:22.04 AS ros2-nomad

# docker build --target ros2-nomad -t ros2-nomad:latest -f nomad200-ros2.Dockerfile .

# docker run -dit --user nomad --name nomad --net=host --ipc=host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix  -e DISPLAY=$DISPLAY  ros2-nomad:latest bash

# docker exec -it nomad /bin/bash

# environment
ENV DISPLAY :0
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
VOLUME /tmp/.X11-unix
ARG DEBIAN_FRONTEND=noninteractive

# install cmake dependencies
RUN apt update && apt install -y \
    sudo \
    build-essential \
    gcc \
    g++ \
    libssl-dev \
    cmake \
    make \
    tar \
    wget \
    nano \
    vim \
    nano \
    apt-utils \
    x11-xserver-utils \
    bash-completion \
    gnupg2 \
    syslinux-utils \
    tmux \
    locales \ 
    openssh-server \
    zip \
    unzip \
    libasio-dev

# set locale
RUN apt update && sudo apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# setup sources
RUN apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && sudo apt install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 packages
RUN apt update && apt upgrade -y && \
	apt install -y ros-humble-desktop && \
	apt install -y ros-dev-tools

# Gazebo Fortress
RUN apt-get update && apt-get install -y lsb-release wget gnupg && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install  -y ignition-fortress && \
    apt-get install -y \
    ros-humble-xacro \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge

# source setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# user nomad
ENV USERNAME nomad
ENV HOME /home/$USERNAME
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        #mkdir /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME
		
SHELL ["/bin/bash", "-c"]
