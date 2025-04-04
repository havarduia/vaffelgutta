import cv2
import numpy as np
from camera.Config.misc import print_blue, print_error, smooth_data as smooth, ConfigLoader

class Aruco:
    def __init__(self, camera, config_loader):
        self.camera = camera
        # A default marker length (if a marker's id is not in the mapping)
        self.marker_length = config_loader.get("marker_length")
        self.marker_sizes = config_loader.get("marker_sizes", {})
        self.dictionary = config_loader.get("dictionary")

        if self.camera is None:
            raise ValueError("Camera instance not found. Ensure Camera is initialized first.")

        self.detector = self._detector()

    def _detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5  # Window size for corner refinement
        parameters.cornerRefinementMaxIterations = 30  # Max iterations for refinement
        parameters.cornerRefinementMinAccuracy = 0.01  # Minimum accuracy for refinement
        parameters.polygonalApproxAccuracyRate = 0.03
        parameters.relativeCornerRefinmentWinSize = 0.15
        parameters.minCornerDistanceRate = 0.05
        parameters.minDistanceToBorder = 3
        parameters.useAruco3Detection = True
        
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)

    @staticmethod
    def get_points(corner, marker_length):
        objectPoints = np.array([
            [-marker_length / 2,  marker_length / 2, 0],
            [ marker_length / 2,  marker_length / 2, 0],
            [ marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0],
        ], dtype=np.float32)
        imagePoints = np.array(corner, dtype=np.float32).reshape(-1, 2)
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
                    np.array(corners[i], dtype=np.float32),
                    (5, 5),
                    (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                )

        return corners, ids.flatten() if ids is not None else None

    def get_homo_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=R.dtype)
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
            # Choose marker size for this tag id if available; otherwise, use default.
            marker_length = self.marker_sizes.get(tag_id, self.marker_length)
            objectPoints, imagePoints = self.get_points(corner, marker_length)

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
        rvecs, tvecs = smooth(np.array(rvecs)), smooth(np.array(tvecs))

        return {tag_id: self.get_homo_matrix(rvec, tvec).tolist() 
                for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs)}

