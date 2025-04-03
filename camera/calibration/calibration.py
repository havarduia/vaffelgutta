import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Charuco Board Configuration
CHARUCO_ROWS = 7  # Number of squares in height
CHARUCO_COLS = 5  # Number of squares in width
SQUARE_LENGTH = 0.04  # In meters
MARKER_LENGTH = 0.03  # In meters
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # Aruco dictionary type


# Create the Charuco board and detector
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard((CHARUCO_COLS, CHARUCO_ROWS), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
charuco_detector = cv2.aruco.CharucoDetector(board)

# Initialize RealSense Camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
pipeline.start(config)

# Storage for calibration
all_corners = []
all_ids = []
image_size = None

# Timing and control variables
capture_mode = False
last_capture_time = time.time()
capture_interval = 1.5  # seconds
target_frames = 50  # Number of frames to capture

print("[INFO] Waiting 5 seconds before starting automatic frame capture...")
time.sleep(5)
capture_mode = True
print("[INFO] Started capturing frames every 2 seconds.")

try:
    while len(all_corners) < target_frames:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect markers and interpolate Charuco corners
        charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)
        
        if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
            cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
                
        cv2.imshow("Calibration", image)
        key = cv2.waitKey(1) & 0xFF
        
        if capture_mode and (time.time() - last_capture_time) >= capture_interval:
            if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 0:
                print(f"[INFO] Captured frame {len(all_corners) + 1}/{target_frames} for calibration.")
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                image_size = gray.shape[::-1]  # Get (width, height)
                last_capture_time = time.time()
        
        elif key == ord('q'):
            print("[INFO] Manual termination detected. Proceeding to calibration with captured frames.")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# Perform camera calibration
if len(all_corners) >= 10:  # Ensure sufficient frames
    print("[INFO] Calibrating camera...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, board, image_size, None, None
    )

    
    print("[INFO] Calibration complete.")
    print("Camera Matrix:")
    print(camera_matrix)
    print("Distortion Coefficients:")
    print(dist_coeffs)
    
    # Save calibration results
    np.savez("camera_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print("[INFO] Calibration data saved to camera_calibration.npz.")
else:
    print("[ERROR] Not enough valid frames for calibration. Try capturing more frames.")
