import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

class Aruco:
    def __init__(self):
        
        # Define camera serial numbers and their resolutions.
        camera_configs = {
            "031422250347": (1280, 720),
            "912112072861": (1920, 1080)
        }

        # Start cameras as None.
        self.cameras = {}

        # Get the list of connected cameras from get_connected_cameras under.
        connected_serials = self.get_connected_cameras()

        # Try to initialize it if it's connected.
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

        # Raise an error.
        if not any(info["obj"] for info in self.cameras.values()):
            raise RuntimeError("No RealSense cameras detected!")

    def get_connected_cameras(self):
        # Return a list of connected cameras.
        return [device.get_info(rs.camera_info.serial_number) for device in rs.context().devices]

    def detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def get_frame(self, camera):
        frame = camera.get_frame()
        return camera.np_frames(frame)

    def aruco_detection(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_detector = self.detector()
        corners, ids, rejected = aruco_detector.detectMarkers(gray)
        return corners, ids, rejected

    def estimate_pose(self, marker_length=0.048):
        def process_camera(camera_info, cam_name):
            camera, matrix, coeff = camera_info["obj"], camera_info["matrix"], camera_info["coeffs"]
            if camera is None:
                return None, []

            image = self.get_frame(camera)
            corners, ids, rejected = self.aruco_detection(image)

            if ids is not None:
                object_points = np.array([
                    [-marker_length/2,  marker_length/2, 0],
                    [ marker_length/2,  marker_length/2, 0],
                    [ marker_length/2, -marker_length/2, 0],
                    [-marker_length/2, -marker_length/2, 0]
                ], dtype=np.float32)

                transformations = []

                for i in range(len(ids)):
                    
                    img_points = corners[i].reshape(-1, 2)
                    retval, rvec, tvec = cv2.solvePnP(object_points, img_points, matrix, coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    
                    if not retval:
                        continue  # Retval checks if marker is valid.
            
                    R, _ = cv2.Rodrigues(rvec)
            
                    # Custom rotation correction
                    R_custom = np.array([
                        [0, -1, 0],   
                        [0, 0, 1],    
                        [-1, 0, 0]    
                    ], dtype=np.float32)
            
                    R_rotated = R @ R_custom  
            
                    T = np.eye(4)
                    T[:3, :3] = R_rotated
                    T[:3, 3] = tvec.flatten()
            
                    transformations.append((ids[i][0], T))
            
                    # Draw axes
                    rvec_rotated, _ = cv2.Rodrigues(R_rotated)
                    cv2.drawFrameAxes(image, matrix, coeff, rvec_rotated, tvec, 0.03)

            return image, transformations

        return {cam_id: process_camera(info, f"Camera {i+1}") 
                for i, (cam_id, info) in enumerate(self.cameras.items())}


# This is only for debugging
def main():
    aruco = Aruco()
    
    while True:
        try:
            pose_data = aruco.estimate_pose()

            for cam_id, (image, poses) in pose_data.items():
                if image is not None:
                    cv2.imshow(f"Camera {cam_id}", image)

                print(f"\n--- Camera {cam_id} ---")
                for marker_id, T in poses:
                    print(f"\nMarker ID: {marker_id}")
                    
                    # Extract components from transformation matrix
                    R_rotated = T[:3, :3]
                    tvec = T[:3, 3]

                    # Print coordinate system verification
                    print("X-axis (inward):", R_rotated[:, 0])
                    print("Y-axis (left):  ", R_rotated[:, 1])
                    print("Z-axis (up):    ", R_rotated[:, 2])
                    print("Translation:    ", tvec)
        finally:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
