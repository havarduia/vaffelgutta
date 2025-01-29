from os import killpg, getpgid, setpgrp
from subprocess import Popen 
from signal import SIGTERM
from time import sleep
import os
import subprocess

# Global variable to store process ID
aruco_process_pid = None

VENV_PATH = os.path.expanduser("~/git/vaffelgutta/camera/venv")
ROS_SETUP_PATH = "/opt/ros/humble/setup.bash"
WS_SETUP_PATH = "~/git/vaffelgutta/camera/camera_ws/install/setup.bash"
LAUNCH_CMD = "ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py"

def camera_launch():
    """Set up virtual environment, install required packages, and launch ArUco pose estimation."""
    global aruco_process_pid

    # Create virtual environment if it doesn't exist
    if not os.path.exists(VENV_PATH):
        print("Creating virtual environment...")
        subprocess.run(["python3", "-m", "venv", VENV_PATH])
    
    pip_path = os.path.join(VENV_PATH, "bin", "pip")

    # Install required packages
    print("Installing required packages...")
    subprocess.run([pip_path, "install", "--upgrade", "pip"])
    subprocess.run([pip_path, "install", "numpy<2", "opencv-python", "opencv-contrib-python", "transforms3d"])

    print("Virtual environment is ready with required packages.")

    # Launch the ROS 2 ArUco pose estimation node only if it’s not running
    command = (
        f'source {ROS_SETUP_PATH} && '
        f'source {WS_SETUP_PATH} && '
        f'source {VENV_PATH}/bin/activate && '
        f'{LAUNCH_CMD}'
    )

    process = Popen(["bash", "-c", command], preexec_fn=setpgrp)
    aruco_process_pid = process.pid
    sleep(0.5)  # Allow some time for the process to start
    print(f"Aruco Pose Estimation: Process started with PID {process.pid}")

def camera_close():
    """Terminate the ArUco pose estimation node."""
    global aruco_process_pid

    # Close the ArUco Pose Estimation process if running
    if aruco_process_pid is not None:
        try:
            killpg(getpgid(aruco_process_pid), SIGTERM)
            print("Aruco Pose Estimation: Process killed") 
        except Exception as e:
            print(f"Error while killing the process: {e}")
        finally:
            print("Aruco Pose Estimation: Shutdown complete")
    else:
        print("Aruco Pose Estimation: No running process to stop.")
