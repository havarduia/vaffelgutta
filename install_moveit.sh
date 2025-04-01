#!/bin/bash

set -e  # Exit on error

cd ~/

# Install necessary dependencies
sudo apt update
sudo apt install -y python3-rosdep

# Initialize and update rosdep
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Upgrade system packages
sudo apt update
sudo apt dist-upgrade -y

# Install colcon and mixin support
sudo apt install -y python3-colcon-common-extensions python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || echo "colcon mixin already added"
colcon mixin update default

# Install vcstool
sudo apt install -y python3-vcstool

# Create workspace and clone MoveIt tutorials
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble https://github.com/moveit/moveit2_tutorials

# Import dependencies
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos

# Install ROS dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

# Ask user if they want to use --executor sequential
read -p "Do you want to use '--executor sequential' for colcon build? (y/n) [Recommended for stability] " use_sequential
if [[ "$use_sequential" == "y" ]]; then
    EXECUTOR_OPTION="--executor sequential"
else
    EXECUTOR_OPTION=""
fi

# Build MoveIt2 workspace
cd ~/ws_moveit
colcon build --mixin release $EXECUTOR_OPTION

# Source workspace setup on every new shell session
echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

cd ~/git/vaffelgutta
