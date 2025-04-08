import numpy as np
import os
import pyrealsense2 as rs
from camera.Config.misc import ConfigLoader
from camera.realsense import Camera  # Replace with your actual module name
from hand_detection import HandDetector  # Replace with correct file if needed
from robot.tools.file_manipulation import Jsonreader

if __name__ == "__main__":

    camera = Camera()
    json_writer = Jsonreader("your/custom/path/")
    filename = "hand_transform"

    detector = HandDetector(camera, json_writer, filename)
    detector.start()
