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
        parameters = cv2.aruco.DetectorParameters()
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

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if corners:
            # Refine corner positions to sub-pixel accuracy
            for i in range(len(corners)):
                cv2.cornerSubPix(
                    gray,
                    np.array(corners[i], dtype=np.float32),
                    (5, 5),
                    (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.1),
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
        Draw a 3D cube on the marker.

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
        cube_points = np.float32([
            # Base (same plane as marker)
            [-half_length, half_length, 0],  # Front left
            [half_length, half_length, 0],   # Front right
            [half_length, -half_length, 0],  # Back right
            [-half_length, -half_length, 0], # Back left

            # Top of the cube
            [-half_length, half_length, cube_height],  # Front left top
            [half_length, half_length, cube_height],   # Front right top
            [half_length, -half_length, cube_height],  # Back right top
            [-half_length, -half_length, cube_height]  # Back left top
        ])

        # Project 3D points to the image plane
        cube_image_points, _ = cv2.projectPoints(
            cube_points, rvec, tvec, camera_matrix, dist_coeffs
        )
        cube_image_points = cube_image_points.reshape(-1, 2).astype(int)

        # Define edges of the cube
        edges = [
            # Base
            (0, 1), (1, 2), (2, 3), (3, 0),
            # Top
            (4, 5), (5, 6), (6, 7), (7, 4),
            # Connecting base to top
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        # Draw edges
        for i, j in edges:
            pt1 = tuple(cube_image_points[i])
            pt2 = tuple(cube_image_points[j])
            cv2.line(image, pt1, pt2, (0, 255, 255), 2)  # Yellow lines

        # Draw vertices
        for point in cube_image_points:
            cv2.circle(image, tuple(point), 4, (0, 0, 255), -1)  # Red circles

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

        pose_image = image.copy() if (draw_markers or draw_cubes) else None
        corners, ids = self.detect_markers(image)

        if ids is None or not corners:
            # No markers detected
            return {}, pose_image

        raw_poses = []
        for tag_id, corner in zip(ids, corners):
            marker_length = self.marker_sizes.get(tag_id, self.marker_length)
            objectPoints, imagePoints = self._get_points(corner, marker_length)

            success, rvec, tvec = cv2.solvePnP(
                objectPoints,
                imagePoints,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if not success:
                # Pose estimation failed for this tag
                continue

            if pose_image is not None:
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
                    cv2.aruco.drawDetectedMarkers(pose_image, corners, ids)

                if draw_cubes:
                    # Draw 3D cube on the marker
                    self._draw_cube(
                        pose_image,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        marker_length
                    )

            raw_poses.append((tag_id, rvec, tvec))

        if not raw_poses:
            return {}, pose_image

        poses = {
            tag_id: self._get_homo_matrix(rvec, tvec).tolist()
            for tag_id, rvec, tvec in raw_poses
        }

        return poses, pose_image
