import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

class Aruco:
    def __init__(self):
        # Camera serial numbers
        self.cameras = {
            "031422250347": {"obj": None, "resolution": (1280, 720)},
            "912112072861": {"obj": None, "resolution": (1920, 1080)}
        }

        # Get list of connected cameras
        connected_cameras = self.get_connected_cameras()

        # Initialize cameras if connected
        for cam_id, info in self.cameras.items():
            if cam_id in connected_cameras:
                info["obj"] = Camera(cam_id, *info["resolution"])
                info["matrix"] = info["obj"].camera_matrix
                info["coeffs"] = info["obj"].dist_coeffs
            else:
                info["matrix"] = info["coeffs"] = None

        if not any(info["obj"] for info in self.cameras.values()):
            raise RuntimeError("No RealSense cameras detected!")

    def get_connected_cameras(self):
        """Returns a list of serial numbers for all connected cameras"""
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

            # Draw detected markers on the frame
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)

            # Estimate pose if markers detected
            transformations = []
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, matrix, coeff)
                for i in range(len(ids)):
                    # Convert rotation vector to matrix
                    R, _ = cv2.Rodrigues(rvecs[i])

                    angle = np.pi / 2  # 90 degrees in radians (adjust as needed)
                    rotation_axis = [0, 0, 1] # X Y Z
                    R_custom, _ = cv2.Rodrigues(np.array(rotation_axis) * angle)
                    R_rotated = R @ R_custom

                    # Create the 4x4 transformation matrix
                    T = np.eye(4)
                    T[:3, :3] = R_rotated
                    T[:3, 3] = tvecs[i].flatten() 

                    transformations.append((ids[i][0], T))
                    # Draw the axes with the rotated pose
                    rvec_rotated, _ = cv2.Rodrigues(R_rotated)  # Convert back to vector for drawing
                    cv2.drawFrameAxes(image, matrix, coeff, rvec_rotated, tvecs[i], 0.03)

            return image, transformations

        # Process cameras and get images with pose data
        results = {cam_id: process_camera(info, f"Camera {i+1}") for i, (cam_id, info) in enumerate(self.cameras.items())}
        return results







# This is only for debugging
def main():
    aruco = Aruco()
    
    while True:
        pose_data = aruco.estimate_pose()

        for cam_id, (image, poses) in pose_data.items():
            if image is not None:
                cv2.imshow(f"Camera {cam_id}", image)

            print(f"\n--- Camera {cam_id} ---")
            for marker_id, T in poses:
                print(f"Marker ID: {marker_id}")
                print(np.array2string(T, precision=4, suppress_small=True))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
