#!/bin/bash

# Detect the Ubuntu codename
UBUNTU_CODENAME=$(lsb_release -sc)

# Function to set locale
set_locale() {
    sudo apt update
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
}

# Function to add ROS 2 apt repository
add_ros2_repository() {
    sudo apt update && sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release

    # Add the ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # For Ubuntu 25.04, use the 'noble' (24.04) repository
    if [ "$UBUNTU_CODENAME" = "plucky" ]; then
        echo "Ubuntu 25.04 detected. Attempting to use ROS 2 Jazzy packages from Ubuntu 24.04 (noble) repository."
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    else
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    fi
}

# Function to install ROS 2 packages
install_ros2_packages() {
    sudo apt update
    sudo apt install -y \
      ros-jazzy-desktop \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-argcomplete \
      python3-vcstool \
      build-essential
}

# Function to initialize rosdep
initialize_rosdep() {
    sudo rosdep init
    rosdep update
}

# Function to setup environment
setup_environment() {
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Function to install development tools (optional)
install_dev_tools() {
    sudo apt update && sudo apt install -y ros-dev-tools
}

# Begin installation
echo "Starting ROS 2 Jazzy installation..."

# Set locale
set_locale

# Add ROS 2 repository
add_ros2_repository

# Install ROS 2 packages
install_ros2_packages

# Initialize rosdep
initialize_rosdep

# Setup environment
setup_environment

# Install development tools
install_dev_tools

echo "ROS 2 Jazzy installation attempt completed."
