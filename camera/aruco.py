import pyrealsense2 as rs
import numpy as numphy
import cv2
from camera.camera import Camera

class Aruco:
    def __init__(self):
        # Define camera serial numbers and their resolutions.
        camera_configs = {
            "031422250347": (1280, 720),
            "912112072861": (1280, 720)
        }

        # Start cameras as None.
        self.cameras = {}

        # Get the list of connected cameras.
        connected_serials = self.get_connected_cameras()

        # Try to initialize each camera if it's connected.
        for serial, resolution in camera_configs.items():
            if serial in connected_serials:
                cam = Camera(serial, *resolution)
                self.cameras[serial] = {
                    "obj": cam,
                    "matrix": cam.camera_matrix,
                    "coeffs": cam.dist_coeffs,
                }
            else:
                self.cameras[serial] = {"obj": None, "matrix": None, "coeffs": None}

        # Raise an error if no cameras were detected.
        if not any(info["obj"] for info in self.cameras.values()):
            raise RuntimeError("No RealSense cameras detected!")

    def get_connected_cameras(self):
        # Return a list of connected camera serial numbers.
        return [device.get_info(rs.camera_info.serial_number) for device in rs.context().devices]

    def detector(self):
        # Initialize and return the ArUco detector.
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def aruco_detection(self, image):
        # Convert image to grayscale and detect markers.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_detector = self.detector()
        corners, ids, rejected = aruco_detector.detectMarkers(gray)
        return corners, ids, rejected

    def estimate_pose(self, marker_length=0.048):
        def process_camera(camera_info, cam_name):
            camera, matrix, coeff = camera_info["obj"], camera_info["matrix"], camera_info["coeffs"]
            if camera is None:
                return None, []

            # Use the new combined method to get the color image.
            image = camera.get_image()
            corners, ids, rejected = self.aruco_detection(image)
            
            transformations = []
            
            if ids is not None:
                # Define object points for the marker corners.
                object_points = numphy.array([
                    [-marker_length/2,  marker_length/2, 0],
                    [ marker_length/2,  marker_length/2, 0],
                    [ marker_length/2, -marker_length/2, 0],
                    [-marker_length/2, -marker_length/2, 0]
                ], dtype=numphy.float32)

                for i in range(len(ids)):
                    img_points = corners[i].reshape(-1, 2)
                    retval, rvec, tvec = cv2.solvePnP(object_points, img_points, matrix, coeff,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not retval:
                        continue  # Skip if marker detection failed.
            
                    R, _ = cv2.Rodrigues(rvec)
            
                    # Custom rotation correction.
                    R_custom = numphy.array([
                        [0, -1, 0],
                        [0,  0, 1],
                        [-1, 0, 0]
                    ], dtype=numphy.float32)
                    R_rotated = R @ R_custom  
            
                    T = numphy.eye(4)
                    T[:3, :3] = R_rotated
                    T[:3, 3] = tvec.flatten()
            
                    transformations.append((ids[i][0], T))
            
                    # Draw axes on the image.
                    rvec_rotated, _ = cv2.Rodrigues(R_rotated)
                    cv2.drawFrameAxes(image, matrix, coeff, rvec_rotated, tvec, 0.03)

            return image, transformations

        return {cam_id: process_camera(info, f"Camera {i+1}") 
                for i, (cam_id, info) in enumerate(self.cameras.items())}
