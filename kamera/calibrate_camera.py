import cv2
import numpy as np

def calibrate_camera(images, pattern_size=(8, 6), square_size=25):
    """
    Calibrate a camera using checkerboard images.
    
    Args:
        images: List of file paths to checkerboard images.
        pattern_size: Tuple indicating the number of inner corners per row and column (e.g., (8, 6)).
        square_size: Size of one square in the checkerboard (in millimeters or meters).

    Returns:
        camera_matrix: Intrinsic camera matrix.
        dist_coeffs: Distortion coefficients.
    """
    # Prepare object points (3D points of checkerboard in real world)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points
    objpoints = []
    imgpoints = []

    for image_path in images:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners for visualization
            img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.imshow('Checkerboard', img)
            cv2.waitKey(500)
    
    cv2.destroyAllWindows()

    # Calibrate the camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not ret:
        raise RuntimeError("Calibration failed. Check your images or pattern size.")

    return camera_matrix, dist_coeffs
