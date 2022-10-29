#!/bin/bash
export ROS_DISTRO=foxy

# Check if ROS2 is installed
if [ ! -d "/opt/ros/$ROS_DISTRO/" ]; then
    echo "Installing ROS2 $ROS_DISTRO"
    # Check if it is running as ROOT
    if [ "$EUID" -ne 0 ]
        then echo "Please run as root"
        exit
    fi
    # Set locate configuration
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    #install dependencies 
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    # Get keys
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    # add repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    #install ROS2
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-desktop
    # Install tools
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    pip3 install pytest-rerunfailures
    sudo apt install -y python3-colcon-common-extensions

    rosdep update
    sudo rosdep init

    if [ ! -d "/opt/ros/$ROS_DISTRO/" ]; then
        echo "ROS2 $ROS_DISTRO cannot be installed! check your Ubuntu distribution"
        exit
    fi   
fi

# Check if the XRCEAgent is installed 
if ! command -v MicroXRCEAgent &> /dev/null
then
    echo "Installing micro-ROS agent"
    # Check if it is running as ROOT
    if [ "$EUID" -ne 0 ]
        then echo "Please run as root"
        exit
    fi
    # Clone, compile and install   XRCEAgent
    pushd ~/ > /dev/null
        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        cd Micro-XRCE-DDS-Agent
        mkdir build
        pushd build/ > /dev/null
            cmake ..
            make -j8
            sudo make install
            sudo ldconfig
        popd > /dev/null
    popd > /dev/null
fi

# Check if ARM GCC compiler is installed
if ! command -v arm-none-eabi-gcc &> /dev/null
then
    echo "Installing GCC ARM compiler"
    if [ "$EUID" -ne 0 ]
        then echo "Please run as root"
        exit
    fi
    sudo apt update 
    sudo apt install -y gcc-arm-none-eabi
fi


if [ ! -d "/opt/microros_ws" ]; then
    echo "Installing microROS setup"
    # Setup ROS2 environment
    source /opt/ros/$ROS_DISTRO/setup.bash
    mkdir -p /opt/microros_ws 
    pushd /opt/microros_ws > /dev/null
        git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup 2>/dev/null
        rosdep update && rosdep install --from-path src --ignore-src -y
        sudo apt install -y python3-pip
        colcon build
    popd > /dev/null
    sudo chmod -R +rwx /opt/microros_ws/
fi

source /opt/microros_ws/install/setup.bash