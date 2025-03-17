"""Main script for initializing and running the camera system."""
from camera.realsense import Camera
from camera.aruco import Aruco
from camera.coordinatesystem import CoordinateSystem
from camera.camera_config_loader import ConfigLoader
import cv2
from camera.init_camera import initalize_system as init

def main():
    """Main loop for updating pose estimation and saving data."""
    camera, aruco, coord_sys = init()
    
    while True:
        # Update pose estimation
        coord_sys.save_to_json(25)
if __name__ == '__main__':
    main()
    