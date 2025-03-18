"""Main script for initializing and running the camera system."""
from camera.realsense import Camera
from camera.aruco import Aruco
from camera.coordinatesystem import CoordinateSystem
from camera.misc import ConfigLoader
import cv2
from camera.init_camera import initalize_system as init
import os
from time import sleep
import numpy as numphy
def main():
    """Main loop for updating pose estimation and saving data."""
    camera, aruco, coord_sys = init()  
    while True:
        # Update pose estimation
        transform = coord_sys.transformation_origin_to_tag()
        coord_sys.start(25)
        for tags, T in transform.items():
            print(f"Tag ID {tags} Transformation \n{numphy.array(T)[:3, 3]}\n")
if __name__ == '__main__':
    main()
    