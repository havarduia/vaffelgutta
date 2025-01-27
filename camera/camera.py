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

def drawAxis(img, rvec, tvec, camera_matrix, dist_coeffs, axis_length=0.1):
    axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length], [0, 0, 0]]).reshape(-1, 3)
    img_pts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    origin = tuple(img_pts[3].ravel().astype(int))
    cv2.line(img, origin, tuple(img_pts[0].ravel().astype(int)), (0, 0, 255), 2)  # X-axis (red)
    cv2.line(img, origin, tuple(img_pts[1].ravel().astype(int)), (0, 255, 0), 2)  # Y-axis (green)
    cv2.line(img, origin, tuple(img_pts[2].ravel().astype(int)), (255, 0, 0), 2)  # Z-axis (blue)

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

                # Draw the marker and axes
                cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                drawAxis(color_image, rvec, tvec, camera_matrix, dist_coeffs, 0.1)

                # Create a 4x4 transformation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                transformation_matrix[:3, 3] = tvec.flatten()

                # Swap x (first row, fourth column) and z (third row, fourth column)
                transformation_matrix[0, 3], transformation_matrix[2, 3] = transformation_matrix[2, 3], transformation_matrix[0, 3]

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
