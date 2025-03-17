from camera.realsense import Camera
from camera.filtering import smooth_data as smooth
from camera.vision_instance import InstanceRegistry
from camera.camera_config_loader import ConfigLoader
import cv2
import numpy as numphy
from camera.print import print_blue, print_error

class Aruco:
    def __init__(self):
        # Fetch the Camera instance from the registry.
        self.camera = InstanceRegistry.get("Camera")
        if self.camera is None:
            raise RuntimeError("Camera instance not found. Ensure Camera is initialized before Aruco.")

        # Register this Aruco instance.
        InstanceRegistry.register("Aruco", self)

        # Initialize Aruco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def get_points(self, marker_length, corner):
        """Returns 3D object points and 2D image points for Aruco marker."""
        object_points = numphy.array([
            [-marker_length / 2,  marker_length / 2, 0],
            [ marker_length / 2,  marker_length / 2, 0],
            [ marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0]
        ], dtype=numphy.float32)

        image_points = numphy.array(corner, dtype=numphy.float32).reshape(-1, 2)
        return object_points, image_points

    def _aruco_detection(self):
        """Detects Aruco markers in the camera frame."""
        image = self.camera.get_image()
        if image is None:
            print_error("No image retrieved from camera.")
            return None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        return corners, ids.flatten() if ids is not None else None

    def R_corrected(self, R):
        """Applies correction matrix to the rotation matrix."""
        R = numphy.array(R)  # Use numpy array instead of matrix
        mult = numphy.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ])
        return mult @ R  # Use @ instead of deprecated `*`

    def get_homo_matrix(self, rvec, tvec):
        """Computes the homogeneous transformation matrix."""
        R, _ = cv2.Rodrigues(rvec)
        R = self.R_corrected(R)
        T = numphy.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def estimate_pose(self, config_loader):
        """Estimates pose of detected Aruco markers."""
        marker_length = config_loader.get("marker_length")

        camera_matrix, dist_coeffs = self.camera.get_calibration()
        if camera_matrix is None or dist_coeffs is None:
            print_error("Camera calibration data is missing.")
            return {}

        corners, ids = self._aruco_detection()
        if ids is None or not corners:
            print_error("Marker not detected! 👺")
            return {}

        transformations = {}
        raw_poses = []

        for tag_id, corner in zip(ids, corners):
            object_points, image_points = self.get_points(marker_length, corner)

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                print_error(f"Pose estimation failed for tag {tag_id}.")
                continue

            raw_poses.append((tag_id, rvec, tvec))

        if raw_poses:
            tag_ids, rvecs, tvecs = zip(*raw_poses)
            rvecs = smooth(numphy.array(rvecs))
            tvecs = smooth(numphy.array(tvecs))

            for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs):
                transformations[tag_id] = self.get_homo_matrix(rvec, tvec)

        return transformations
