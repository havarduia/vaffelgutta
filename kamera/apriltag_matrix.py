import pyrealsense2 as rs
import numpy as np
import cv2
import pupil_apriltags as april
from PIL import Image

detector = april.Detector()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Change resoultion and fps here
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of color frames

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())


        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # Show the image
       
        tags = detector.detect(gray_image)
        for tag in tags:
            pts = np.array(tag.corners, dtype=np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(color_image, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
        

        cv2.imshow('AprilTags Detection', color_image)
        k = cv2.waitKey(1) & 0xFF
        # press 'q' to exit
        if k == ord('q'):
            break


finally:
    # Stop streaming
    pipeline.stop()