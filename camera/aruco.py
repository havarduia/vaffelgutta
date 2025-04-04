from camera.realsense import Camera
import cv2
import numpy as numphy
from camera.Config.misc import print_blue, print_error, smooth_data as smooth, ConfigLoader
class Aruco:
    def __init__(self, camera, config_loader):
        # Fetch the Camera instance from the registry.
        self.camera = camera
        self.marker_length = config_loader.get("marker_length")
        if self.camera is None:
            raise Exception(
                "Camera int not found. Ensure Camera is init first."
            )

        self.detector = self._detector()

    def _detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def get_points(self, corner):

        objectPoints = numphy.array(
            [
                [-self.marker_length / 2, self.marker_length / 2, 0],
                [self.marker_length / 2, self.marker_length / 2, 0],
                [self.marker_length / 2, -self.marker_length / 2, 0],
                [-self.marker_length / 2, -self.marker_length / 2, 0],
            ],
            dtype=numphy.float32,
        )

        imagePoints = numphy.array(corner, dtype=numphy.float32).reshape(-1, 2)

        return objectPoints, imagePoints

    def _aruco_detection(self):
        image = self.camera.get_image()
        if image is None:
            return None, None
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            ids = ids.flatten()
        return corners, ids

    def R_corrected(self, R):
        R = numphy.matrix(R)
        mult = numphy.matrix([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        R = mult * R
        return R

    def get_homo_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = numphy.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        R = self.R_corrected(R)
        T[:3, 3] = tvec.flatten()
        return T

    def estimate_pose(self):

        camera_matrix, distcoeffs = self.camera.get_calibration()
        corners, ids = self._aruco_detection()
        if ids is None or len(corners) == 0:
            print_error("Marker not detected! 👺")
            return {}  # Return empty dictionary

        transformations = {}
        raw_poses = []
        for tag_id, corner in zip(ids, corners):

            objectPoints, imagePoints = self.get_points(corner)

            success, rvec, tvec = cv2.solvePnP(
                objectPoints,
                imagePoints,
                camera_matrix,
                distcoeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            
            if not success:
                print_error(f"Pose estimation failed for tag {tag_id}.")
                continue

            # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distcoeffs)
            raw_poses.append((tag_id, rvec, tvec))
        
        if raw_poses:
            tag_ids, rvecs, tvecs = zip(*raw_poses)
            rvecs = smooth(numphy.array(rvecs))
            tvecs = smooth(numphy.array(tvecs))

            for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs):
                T = self.get_homo_matrix(rvec, tvec)
                transformations[tag_id] = T
        return transformations
    
