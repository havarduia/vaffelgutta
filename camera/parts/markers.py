"""
Marker detection and pose estimation functionality.
"""

import cv2
import numpy as np
from camera.config.configloader import ConfigLoader

class Aruco:
    """
    Class for detecting ArUco markers and estimating their poses.
    """

    def __init__(self, camera):
        """
        Initialize the marker detector.

        Args:
            camera: A Camera instance that provides frames and calibration data
        """
        self.camera = camera
        self.config_loader = ConfigLoader()

        # ArUco setup
        self.marker_length = self.config_loader.get("marker_length")
        self.marker_sizes = self.config_loader.get("marker_sizes", {})
        self.dictionary = self.config_loader.get("dictionary")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.dictionary)
        )

        # Configure detector parameters for better performance
        parameters = cv2.aruco.DetectorParameters()
        # Reduce the number of iterations in the corner refinement
        parameters.cornerRefinementMaxIterations = 15
        # Increase the threshold for faster detection
        parameters.adaptiveThreshConstant = 10
        # Reduce the number of marker detection iterations
        parameters.maxMarkerPerimeterRate = 4.0
        # Increase minimum marker distance to avoid false positives
        parameters.minMarkerDistanceRate = 0.05

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

    def detect_markers(self, image=None):
        """
        Detect ArUco markers in the image.

        Args:
            image: Optional image to use. If None, gets a new frame from the camera.

        Returns:
            tuple: (corners, ids) where corners are the marker corners and ids are the marker IDs
        """
        if image is None:
            image = self.camera.get_color_frame()

        if image is None:
            return None, None

        # Convert to grayscale for faster processing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect markers without refinement first
        corners, ids, _ = self.detector.detectMarkers(gray)

        # Only refine corners if markers are detected and we have a reasonable number
        if corners and len(corners) <= 5:  # Only refine if we have 5 or fewer markers
            # Refine corner positions to sub-pixel accuracy with optimized parameters
            for i in range(len(corners)):
                cv2.cornerSubPix(
                    gray,
                    np.array(corners[i], dtype=np.float32),
                    (3, 3),  # Smaller window size
                    (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, 0.1),  # Fewer iterations
                )

        return corners, ids.flatten() if ids is not None else None

    @staticmethod
    def _get_points(corner, marker_length):
        """
        Get 3D object points and 2D image points for pose estimation.

        Args:
            corner: Corner points of the marker
            marker_length: Length of the marker side in meters

        Returns:
            tuple: (objectPoints, imagePoints) for pose estimation
        """
        objectPoints = np.array(
            [
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0],
            ],
            dtype=np.float32,
        )
        imagePoints = np.array(corner, dtype=np.float32).reshape(-1, 2)
        return objectPoints, imagePoints

    @staticmethod
    def _get_homo_matrix(rvec, tvec):
        """
        Convert rotation vector and translation vector to homogeneous transformation matrix.

        Args:
            rvec: Rotation vector
            tvec: Translation vector

        Returns:
            numpy.ndarray: 4x4 homogeneous transformation matrix
        """
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        T[:3, 3] = tvec.ravel()
        return T

    def _draw_cube(self, image, camera_matrix, dist_coeffs, rvec, tvec, marker_length):
        """
        Draw a simplified 3D cube on the marker for better performance.

        Args:
            image: Image to draw on
            camera_matrix: Camera matrix
            dist_coeffs: Distortion coefficients
            rvec: Rotation vector
            tvec: Translation vector
            marker_length: Length of the marker side in meters

        Returns:
            numpy.ndarray: Image with cube drawn on it
        """
        # Define cube points (3D coordinates relative to marker center)
        cube_height = marker_length  # Height of the cube
        half_length = marker_length / 2

        # Define the 8 vertices of the cube
        cube_points = np.float32(
            [
                # Base (same plane as marker)
                [-half_length, half_length, 0],  # Front left
                [half_length, half_length, 0],  # Front right
                [half_length, -half_length, 0],  # Back right
                [-half_length, -half_length, 0],  # Back left
                # Top of the cube
                [-half_length, half_length, cube_height],  # Front left top
                [half_length, half_length, cube_height],  # Front right top
                [half_length, -half_length, cube_height],  # Back right top
                [-half_length, -half_length, cube_height],  # Back left top
            ]
        )

        # Project 3D points to the image plane
        cube_image_points, _ = cv2.projectPoints(
            cube_points, rvec, tvec, camera_matrix, dist_coeffs
        )
        cube_image_points = cube_image_points.reshape(-1, 2).astype(int)

        # Define edges of the cube
        edges = [
            # Base
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            # Top
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            # Connecting base to top
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ]

        # Draw edges with thinner lines for better performance
        for i, j in edges:
            pt1 = tuple(cube_image_points[i])
            pt2 = tuple(cube_image_points[j])
            cv2.line(image, pt1, pt2, (0, 255, 255), 1)  # Thinner yellow lines

        # Skip drawing vertices for better performance
        # Only draw the top center point as a reference
        top_center = tuple(np.mean(cube_image_points[4:8], axis=0).astype(int))
        cv2.circle(image, top_center, 3, (0, 0, 255), -1)  # Red circle

        return image

    def estimate_poses(self, image=None, draw_markers=True, draw_cubes=True):
        """
        Estimate poses of all detected markers.

        Args:
            image: Optional image to use. If None, gets a new frame from the camera.
            draw_markers: Whether to draw markers and axes on the image
            draw_cubes: Whether to draw 3D cubes on the markers

        Returns:
            tuple: (poses, annotated_image) where poses is a dictionary mapping marker IDs to poses
        """
        camera_matrix, dist_coeffs = self.camera.get_calibration()

        if image is None:
            image = self.camera.get_color_frame()

        if image is None:
            return {}, None

        # Only create a copy if we need to draw on it
        pose_image = image.copy() if (draw_markers or draw_cubes) else None

        # Detect markers
        corners, ids = self.detect_markers(image)

        if ids is None or not corners:
            # No markers detected
            return {}, pose_image

        # Process all markers at once
        raw_poses = []
        for tag_id, corner in zip(ids, corners):
            marker_length = self.marker_sizes.get(tag_id, self.marker_length)
            objectPoints, imagePoints = self._get_points(corner, marker_length)

            # Use a faster PnP method with fewer iterations
            success, rvec, tvec = cv2.solvePnP(
                objectPoints,
                imagePoints,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,  # Fast method for square markers
            )

            if not success:
                # Pose estimation failed for this tag
                continue

            raw_poses.append((tag_id, rvec, tvec))

        # Only draw if we have poses and need to draw
        if pose_image is not None and raw_poses:
            # Draw all markers at once if needed
            if draw_markers and corners is not None and ids is not None:
                cv2.aruco.drawDetectedMarkers(pose_image, corners, ids)

            # Draw axes and cubes for each marker
            for tag_id, rvec, tvec in raw_poses:
                marker_length = self.marker_sizes.get(tag_id, self.marker_length)

                if draw_markers:
                    # Draw axis on the marker
                    cv2.drawFrameAxes(
                        pose_image,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        marker_length * 0.5,
                    )

                if draw_cubes:
                    # Draw 3D cube on the marker
                    self._draw_cube(
                        pose_image,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        marker_length,
                    )

        if not raw_poses:
            return {}, pose_image

        # Create transformation matrices for all poses at once
        poses = {
            tag_id: self._get_homo_matrix(rvec, tvec).tolist()
            for tag_id, rvec, tvec in raw_poses
        }

        return poses, pose_image
