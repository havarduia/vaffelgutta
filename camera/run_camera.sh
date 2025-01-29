#!/bin/bash

# Activate the virtual environment
source ~/git/vaffelgutta/camera/venv/bin/activate

# Source the ROS2 workspace setup
source ~/git/vaffelgutta/camera/camera_ws/install/setup.bash

# Launch the ROS2 package
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
