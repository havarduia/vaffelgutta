
# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

import pyrealsense2 as rs
import cv2
import numpy as np

# Camera intrinsic parameters
camera_matrix = np.array([
    [960.981, 0, 956.882],
    [0, 960.981, 529.997],
    [0, 0, 1]
])
dist_coeffs = np.array([0, 0, 0, 0, 0])  # Update if distortion coefficients are available

# Configure the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# Load ArUco marker dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                # Estimate pose of the ArUco marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.035, camera_matrix, dist_coeffs)
                print(tvec)
                tvec = tvec[0][0]
                x = tvec[0]; y = tvec[1]; z = tvec[2]
             
                x_new = z
                y_new = -x
                z_new = -y 
                tvec = np.array([[[x_new, y_new, z_new]]])   
                print(tvec)
                # Draw the marker and axes
                cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, tvec, length=0.05)

                # Create a 4x4 transformation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                transformation_matrix[:3, 3] = tvec.flatten()


                with open("robot_workspace/assets/camera_readings.py", "a") as file:
                    file.write(f"\ntest_marker =([\n")
                    np.savetxt(file, transformation_matrix, fmt="  [% .8f, % .8f, % .8f, % .8f],")
                    file.write("  ])\n")
                # Print the transformation matrix in the desired format
                print(f"([")
                for row in transformation_matrix:
                    print(f"  [{', '.join(f'{x: .8f}' for x in row)}],")
                print(f" ])\n")

        else:
            print("No ArUco markers detected")

        # Display the result
        cv2.imshow('RealSense', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close all OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()
