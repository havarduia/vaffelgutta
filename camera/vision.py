import pyrealsense2 as rs
import numpy as np
import cv2
from camera.config.configloader import ConfigLoader
from robot.tools.file_manipulation import Jsonreader
from typing import Union
class Vision:
    def __init__(self):
        self.config_loader = ConfigLoader()
        self.jsonreader = Jsonreader()

        # -----------------------
        # Camera Setup
        # -----------------------
        self.isStreaming = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Find the first available RealSense camera.
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("No RealSense camera found.")

        camera_id = devices[0].get_info(rs.camera_info.serial_number)
        print(f"Using RealSense camera with serial: {camera_id}")

        resolution = self.config_loader.get("resolution")
        resolution_d = self.config_loader.get("resolution_d")
        fps = self.config_loader.get("fps")

        self.config.enable_device(camera_id)
        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps
        )
        self.config.enable_stream(
            rs.stream.depth, resolution_d[0], resolution_d[1], rs.format.z16, fps
        )

        self.color_frame = None
        self.intrinsics = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Start the camera streaming and get calibration
        self._start_streaming()

        # -----------------------
        # ArUco Setup
        # -----------------------
        self.marker_length = self.config_loader.get("marker_length")
        self.marker_sizes = self.config_loader.get("marker_sizes", {})
        self.dictionary = self.config_loader.get("dictionary")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.dictionary)
        )
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

        # -----------------------
        # Coordinate System Setup
        # -----------------------
        self.origin_id = self.config_loader.get("origin_id")
        self.offset_x = self.config_loader.get("offset_x")
        self.offset_y = self.config_loader.get("offset_y")
        self.offset_z = self.config_loader.get("offset_z")
        self.bias_x = self.config_loader.get("bias_x")
        self.bias_y = self.config_loader.get("bias_y")
        self.bias_z = self.config_loader.get("bias_z")
        self.last_origin_inv = (
            None  # Store last known inverse transformation from origin
        )

    def __del__(self):
        self._stop_streaming()

    # =====================================================
    # Camera Methods
    # =====================================================
    def _start_streaming(self):
        if not self.isStreaming:
            try:
                profile = self.pipeline.start(self.config)
                # Change sensor settings (for instance, set sharpness for the color sensor)
                color_sensor = profile.get_device().query_sensors()[1]
                color_sensor.set_option(rs.option.sharpness, 100)
            except Exception as e:
                print(f"Error starting pipeline: {e}")
                raise e

            self.isStreaming = True
            # Load additional calibration data if needed
            intrinsics = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
            self.intrinsics = intrinsics
            self.camera_matrix = np.array(
                [
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1],
                ]
            )
            self.dist_coeffs = np.array(intrinsics.coeffs[:5])

    def _stop_streaming(self):
        if self.isStreaming:
            self.pipeline.stop()
            self.isStreaming = False

    def _get_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        image = np.asanyarray(color_frame.get_data())
        return image

    def _get_depth_image(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    def _get_calibration(self):
        """Return the camera matrix and distortion coefficients."""
        return self.camera_matrix, self.dist_coeffs

    # =====================================================
    # ArUco and Pose Estimation Methods
    # =====================================================
    @staticmethod
    def _get_points(corner, marker_length):
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

    def _aruco_detection(self):
        image = self._get_image()
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

    def _get_homo_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        T[:3, 3] = tvec.ravel()
        return T

    def _estimate_pose(self):
        camera_matrix, dist_coeffs = self._get_calibration()
        pose_image = self._get_image()
        corners, ids = self._aruco_detection()

        if ids is None or not corners:
            print("Marker not detected! 👺")
            return {}, None  # Return the original frame even if no detection

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
                print(f"Pose estimation failed for tag {tag_id}.")
                continue

            # Draw axis on the marker
            cv2.drawFrameAxes(
                pose_image, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.5
            )
            
            cv2.aruco.drawDetectedMarkers(pose_image, corners, ids)
            
            raw_poses.append((tag_id, rvec, tvec))

        if not raw_poses:
            return {}, pose_image

        tag_ids, rvecs, tvecs = zip(*raw_poses)

        return {
            tag_id: self._get_homo_matrix(rvec, tvec).tolist()
            for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs)
        }, pose_image

    # =====================================================
    # Coordinate System Methods
    # =====================================================
    def _coordinatesystem(self):
        """
        Computes the transformation from the origin marker
        """

        pose , image = self._estimate_pose()

        if self.origin_id in pose:
            origin_inv = np.linalg.inv(pose[self.origin_id])
            self.last_origin_inv = origin_inv
        else:
            return {}, None

        if origin_inv is None:
            return {}, None

        tags = {}
        for id, pose in pose.items():

            o_to_t = origin_inv @ np.array(pose)
            transformed_matrix = np.array(
                [
                    [
                        o_to_t[1, 1],
                        -o_to_t[1, 0],
                        o_to_t[1, 2],
                        o_to_t[1, 3] + self.bias_x - self.offset_x,
                    ],
                    [
                        -o_to_t[0, 1],
                        o_to_t[0, 0],
                        -o_to_t[0, 2],
                        -o_to_t[0, 3] + self.bias_y - self.offset_y,
                    ],
                    [
                        o_to_t[2, 1],
                        -o_to_t[2, 0],
                        o_to_t[2, 2],
                        o_to_t[2, 3] + self.bias_z - self.offset_z,
                    ],
                    [0, 0, 0, 1],
                ]
            ).tolist()

            tags[id] = transformed_matrix

        return tags, image

    def _init_tags(self, tags: str | int):
        all_tags = []
        for tag in tags:
            all_tags.append(str(tag))

        return all_tags

    from typing import Union

    def run_once(self, *allowed_tags: Union[str, int], return_image: bool = False):
        tags, image = self._coordinatesystem()
        allowed_tags = self._init_tags(allowed_tags)
        self.jsonreader.write("camera_readings", tags)
        data = self.jsonreader.read("camera_readings")
        
        if not len(allowed_tags) == 0:
            for key in list(data.keys()):
                if key not in allowed_tags:
                    print(f"removed hallucinated tag, id: {key}")
                    self.jsonreader.pop("camera_readings", key)
        
        if return_image:
            return image
        
    def run(self, *allowed_tags: str | int, show_image: bool=False):
        """
        Continuously runs pose estimation, writes data, and shows video feed.
        Press ESC to exit.
        """
    
        while True:
            
            img = self.run_once(*allowed_tags, return_image=True)
            
            if show_image == True:
                if img is not None:
                    cv2.imshow("Pose Estimation Feed", img)
                    
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to break
                cv2.destroyAllWindows()
                break        

# =====================================================
# Example Usage
# =====================================================
if __name__ == "__main__":
    vision = Vision()
    vision.run_once()
