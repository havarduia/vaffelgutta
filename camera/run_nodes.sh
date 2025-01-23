#!/bin/bash

#Virtual Workspace camera
gnome-terminal -- bash -c "
source ~/git/vaffelgutta/virtual/camera_env/bin/activate &&
source ~/git/vaffelgutta/camera_library/camera_ws/install/setup.bash &&
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py;
exec bash"




