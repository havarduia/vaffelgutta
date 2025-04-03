import cv2
import numpy as numphy
from camera.Config.misc import print_blue, print_error, smooth_data as smooth, ConfigLoader

class Aruco:
    def __init__(self, camera, config_loader):
        self.camera = camera
        self.marker_length = config_loader.get("marker_length")

        if self.camera is None:
            raise ValueError("Camera instance not found. Ensure Camera is initialized first.")

        self.detector = self._detector()

    def _detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5  # Window size for corner refinement
        parameters.cornerRefinementMaxIterations = 30  # Max iterations for refinement
        parameters.cornerRefinementMinAccuracy = 0.1  # Minimum accuracy for refinement
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)

    @staticmethod
    def get_points(corner, marker_length):
        objectPoints = numphy.array([
            [-marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0],
        ], dtype=numphy.float32)

        imagePoints = numphy.array(corner, dtype=numphy.float32).reshape(-1, 2)
        return objectPoints, imagePoints

    def _aruco_detection(self):
        image = self.camera.get_image()
        if image is None:
            return None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if corners:
            # Refine corner positions to sub-pixel accuracy
            for i in range(len(corners)):
                cv2.cornerSubPix(
                    gray,
                    numphy.array(corners[i], dtype=numphy.float32),
                    (5, 5),
                    (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                )

        return corners, ids.flatten() if ids is not None else None

    def get_homo_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = numphy.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        T[:3, 3] = tvec.ravel()  
        return T


    def estimate_pose(self):
        camera_matrix, distcoeffs = self.camera.get_calibration()
        corners, ids = self._aruco_detection()

        if ids is None or not corners:
            print_error("Marker not detected! 👺")
            return {}  # Return empty dictionary

        raw_poses = []
        for tag_id, corner in zip(ids, corners):
            objectPoints, imagePoints = self.get_points(corner, self.marker_length)

            success, rvec, tvec = cv2.solvePnP(
                objectPoints, imagePoints, camera_matrix, distcoeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if not success:
                print_error(f"Pose estimation failed for tag {tag_id}.")
                continue

            raw_poses.append((tag_id, rvec, tvec))

        if not raw_poses:
            return {}

        tag_ids, rvecs, tvecs = zip(*raw_poses)
        rvecs, tvecs = smooth(numphy.array(rvecs)), smooth(numphy.array(tvecs))

        return {tag_id: self.get_homo_matrix(rvec, tvec) for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs)}
