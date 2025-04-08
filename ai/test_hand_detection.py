import numpy as np
import os
import pyrealsense2 as rs
from camera.Config.misc import ConfigLoader
from camera.realsense import Camera  # Replace with your actual module name
from hand_detection import HandDetector  # Replace with correct file if needed

if __name__ == "__main__":
    config_loader = ConfigLoader()
    cam = Camera(config_loader)
    detector = HandDetector(cam)
    detector.start()