import pyrealsense2 as rs
import numpy as np
import cv2

# ================= Configuration Constants =================
CAMERAS = {
    "camera_1": "912112072861",  # Short-distance camera (closer to tag 25)
    "camera_2": "031422250347"   # Main camera (captures entire table including origin)
}
FRAME_SIZE = (1280, 720)
FPS = 30
MARKER_LENGTH = 0.048  # Size of ArUco marker in meters
ORIGIN_MARKER_ID = 0   # ID of the reference (origin) marker taped on table
TARGET_MARKER_ID = 25  # ID of the target marker
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# Precompute the correction rotation for the origin marker (-90° about the x-axis)
#R_CORRECTION_ORIGIN = cv2.Rodrigues(np.array([0, 0, 0]))[0]


# ================= Camera System Class =================
class CameraSystem:
    def __init__(self):
        """Initialize RealSense cameras and ArUco detection parameters."""
        self.cameras = {}
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.T_origin = None       # Transformation from origin marker to camera_2 frame
        self.T_cam1_to_cam2 = None # Transformation from camera_1 frame to camera_2 frame
        self._initialize_cameras()

    def _initialize_cameras(self) -> None:
        """Initialize pipelines and store intrinsic parameters."""
        for name, serial in CAMERAS.items():
            try:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(rs.stream.color, *FRAME_SIZE, rs.format.bgr8, FPS)
                profile = pipeline.start(config)
                stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                intrinsics = stream.get_intrinsics()

                camera_matrix = np.array([
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1]
                ])
                dist_coeffs = np.array(intrinsics.coeffs[:5])
                self.cameras[name] = {
                    'pipeline': pipeline,
                    'camera_matrix': camera_matrix,
                    'dist_coeffs': dist_coeffs
                }
                print(f"Initialized {name} successfully.")
            except RuntimeError as e:
                print(f"Initialization failed for {name}: {e}")
                continue

    def stop_all(self) -> None:
        """Stop all RealSense pipelines."""
        for cam in self.cameras.values():
            cam['pipeline'].stop()


# ================= Transformation Helper Functions =================
def get_transformation_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """
    Create a 4x4 transformation matrix from rotation (rvec) and translation (tvec) vectors.
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def get_translation_norm(T: np.ndarray) -> float:
    """
    Compute the Euclidean norm of the translation component of a transformation matrix.
    """
    return np.linalg.norm(T[:3, 3])

#Conver to world frame 

def convert_to_world_frame(T: np.ndarray) -> np.ndarray:
    """
    Convert a camera frame to a ROS standard frame.
    """
    T_world = np.array([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    return T_world @ T @ np.linalg.inv(T_world)

# ================= Frame Processing Functions =================
def process_frame(cam_name: str, camera: dict, system: CameraSystem) -> (np.ndarray, dict):
    """
    Capture a frame, detect ArUco markers, and update the system state.
    
    Returns:
        frame: The image frame with markers and axes drawn.
        detections: Dictionary containing the target marker's detection (if found).
    """
    detections = {}
    frameset = camera['pipeline'].wait_for_frames()
    color_frame = frameset.get_color_frame()
    if not color_frame:
        return None, detections

    frame = np.asanyarray(color_frame.get_data())
    corners, ids, _ = cv2.aruco.detectMarkers(frame, system.aruco_dict, parameters=system.aruco_params)
    
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_LENGTH, camera['camera_matrix'], camera['dist_coeffs']
        )
        if rvecs is None or tvecs is None:
            return frame, detections

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = rvecs[i], tvecs[i]
            T_marker = get_transformation_matrix(rvec, tvec)

            # Update the origin transformation for camera_2
            if cam_name == "camera_2" and marker_id == ORIGIN_MARKER_ID:
                T_corr = np.eye(4)
               #T_corr[:3, :3] = R_CORRECTION_ORIGIN
                T_marker_corr = T_marker @ T_corr
                system.T_origin = np.linalg.inv(T_marker_corr)

            # Record detection for the target marker
            if marker_id == TARGET_MARKER_ID:
                detections[cam_name] = {'T': T_marker, 'tvec': tvec}

            cv2.drawFrameAxes(frame, camera['camera_matrix'], camera['dist_coeffs'], rvec, tvec, MARKER_LENGTH)
    return frame, detections


def choose_best_detection(detections: dict, system: CameraSystem) -> (np.ndarray, str):
    """
    Choose the best detection for the target marker among available cameras
    based on the translation norm (i.e. proximity).
    
    Returns:
        T_tag_cam2: Transformation matrix for the target marker in camera_2 frame.
        source: Description of the source used.
    """
    T_tag_cam2 = None
    d_cam2 = np.inf
    d_cam1 = np.inf

    if "camera_2" in detections:
        T_candidate = detections["camera_2"]['T']
        d_cam2 = get_translation_norm(T_candidate)

    if "camera_1" in detections:
        if system.T_cam1_to_cam2 is None and "camera_2" in detections:
            system.T_cam1_to_cam2 = (
                detections["camera_2"]['T'] @ np.linalg.inv(detections["camera_1"]['T'])
            )
            print("Calibrated T_cam1_to_cam2.")
        if system.T_cam1_to_cam2 is not None:
            T_candidate = system.T_cam1_to_cam2 @ detections["camera_1"]['T']
            d_cam1 = get_translation_norm(T_candidate)

    if d_cam1 < d_cam2:
        T_tag_cam2 = system.T_cam1_to_cam2 @ detections["camera_1"]['T']
        source = "camera_1 (converted)"
    elif d_cam2 < np.inf:
        T_tag_cam2 = detections["camera_2"]['T']
        source = "camera_2"
    else:
        source = "None"

    return T_tag_cam2, source


def compute_target_pose(system: CameraSystem, detections: dict) -> (np.ndarray, str):
    """
    Compute the target marker's pose in the ROS coordinate frame based on the chosen detection.
    
    Returns:
        H_ros: Transformation matrix in ROS standard frame.
        source: The source camera used for the detection.
    """
    T_tag_cam2, source = choose_best_detection(detections, system)
    if T_tag_cam2 is None or system.T_origin is None:
        return None, None

    H = system.T_origin @ T_tag_cam2
    H_ros = convert_to_world_frame(H)
    return H_ros, source


# ================= Main Loop =================
def main() -> None:
    """Main loop to capture frames, process detections, and compute target pose."""
    system = CameraSystem()
    try:
        while True:
            tag_detections = {}
            frames = {}

            # Process frames from each camera
            for cam_name, camera in system.cameras.items():
                frame, detections = process_frame(cam_name, camera, system)
                if frame is not None:
                    frames[cam_name] = frame
                tag_detections.update(detections)

            # Compute target marker pose if possible
            H_ros, source = compute_target_pose(system, tag_detections)
            if H_ros is not None:
                print(f"Using {source}:")
                print(np.array2string(H_ros, formatter={'float_kind': lambda x: "%.3f" % x}))
                print()

            # Display frames from all cameras
            for cam_name, frame in frames.items():
                cv2.imshow(cam_name, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        system.stop_all()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
