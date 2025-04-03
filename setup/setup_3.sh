#!/bin/bash

set -e  # Exit on error

current_dir=$(pwd)
script_location=$(pwd)/$0
script_dir=${script_location::(-11)}
cd $script_dir

cp ../robot/assets/misc/vx300s_joint_limits.yaml ~/interbotix_ws/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/joint_limits/vx300s_joint_limits.yaml 
cp ../robot/assets/misc/vx300s_joint_limits.yaml ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit/config/joint_limits/vx300s_joint_limits.yaml 

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

sudo apt remove ros-$ROS_DISTRO-moveit*

# Install ROS dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

# Build MoveIt2 workspace
cd ~/ws_moveit
colcon build --mixin release --executor sequential

# Source workspace setup on every new shell session
echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

cd $script_dir
cd ../robot/robot_controllers/Wafflebot/moveit/cpp_ws/
colcon build    
echo "source $script_dir/../robot/robot_controllers/Wafflebot/moveit/cpp_ws/install/setup.bash" >> ~/.bashrc

cd $script_dir
cd ../robot/assets/boundingboxes/
mkdir publish
cd publish
touch add.json
touch remove.json
cd ../../misc
touch time_recordings.json
cd ../position_data
touch camera_readings.json
touch offsets.json

cd $current_dir
