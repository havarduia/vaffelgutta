import pyrealsense2 as rs
import cv2
import numpy as numphy
import os
from time import sleep
numphy.set_printoptions(suppress=True, precision=4)
from print import print_blue
from print import print_error
from robot.tools.file_manipulation import Jsonreader
from filtering import smooth_data as smooth
class InstanceRegistry:
    _instances = {}

    @classmethod
    def register(cls, key, instance):
        cls._instances[key] = instance

    @classmethod
    def get(cls, key):
        return cls._instances.get(key)

class Camera:
    def __init__(self, camera_id, x, y):

        self.isStreaming = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(camera_id)
        self.config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, 30)
        self.color_frame = None
        self.intrinsics = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.start_streaming()
        # Register this instance so other classes can fetch it.
        InstanceRegistry.register("Camera", self)

    def __del__(self):
        self.stop_streaming()
       
    def start_streaming(self):
        if not self.isStreaming:
            try:
                profile = self.pipeline.start(self.config)
            except Exception as e:
                print_error(f"Error starting pipeline: {e}")
                raise e
            self.isStreaming = True

            # Get intrinsics from the color stream.
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.intrinsics = intrinsics
            self.camera_matrix = numphy.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            self.dist_coeffs = numphy.array(intrinsics.coeffs[:5])
            
    def stop_streaming(self):
        if self.isStreaming:
            self.pipeline.stop()
            self.isStreaming = False
            
    def get_image(self):
        
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        image = numphy.asanyarray(color_frame.get_data())
        return image

    def get_calibration(self):
        """Return the camera matrix and distortion coefficients."""
        return self.camera_matrix, self.dist_coeffs

class Aruco:
    def __init__(self):
        # Fetch the Camera instance from the registry.
        self.camera = InstanceRegistry.get("Camera")
        if self.camera is None:
            raise Exception("Camera instance not found. Ensure Camera is initialized before Aruco.")
        # Register this Aruco instance.
        InstanceRegistry.register("Aruco", self)
        
        self.detector = self._detector()
    
    def _detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)    
    
    def get_points(self, marker_length, corner):
        
        objectPoints = numphy.array([
                [-marker_length / 2,  marker_length / 2, 0],
                [ marker_length / 2,  marker_length / 2, 0],
                [ marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ], dtype=numphy.float32)

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
        mult = numphy.matrix([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
            ])
        R =  mult*R
        return R
        
    def get_homo_matrix(self, rvec,tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = numphy.eye(4, dtype=R.dtype)
        T[:3, :3] = R
        R = self.R_corrected(R)
        T[:3, 3] = tvec.flatten()
        return T

    def estimate_pose(self, marker_length=0.048):

        camera_matrix, distcoeffs = self.camera.get_calibration()
        corners, ids= self._aruco_detection()
        
        if ids is None or len(corners) == 0:
            print_error("Marker not detected! 👺")
            return {}  # Return empty dictionary
        
        transformations = {}   
        raw_poses = []     
        for tag_id, corner in zip(ids, corners):
            
            objectPoints, imagePoints = self.get_points(marker_length, corner)
             
            success, rvec, tvec = cv2.solvePnP(objectPoints,
                                               imagePoints,
                                               camera_matrix,
                                               distcoeffs,
                                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
            
            if not success:
              print_error(f"Pose estimation failed for tag {tag_id}.")
              continue
           
            #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distcoeffs)
            raw_poses.append((tag_id, rvec, tvec))
        
        if raw_poses:
            tag_ids, rvecs, tvecs = zip(*raw_poses)
            rvecs = smooth(numphy.array(rvecs))
            tvecs = smooth(numphy.array(tvecs))
            
            for tag_id, rvec, tvec in zip(tag_ids, rvecs, tvecs):
                T = self.get_homo_matrix(rvec,tvec)
                transformations[tag_id] = T
        return transformations 
class CoordinateSystem:
    
    def __init__(self, marker_length=0.048, origin_id=28):
        
        # Fetch the Aruco instance from the registry.
        self.aruco = InstanceRegistry.get("Aruco")
        if self.aruco is None:
            raise Exception("\033[91mAruco instance not found. Ensure Aruco is initialized after Camera.\033[0m")
        
        self.marker_length = marker_length
        self.origin_id = origin_id
        self.transformations = self.aruco.estimate_pose(self.marker_length)
   
    def transformation_origin_to_tag(self, origin_id=0, offset_x=0, offset_y=0, offset_z=0):
        
        if origin_id not in self.transformations:
            print_error(f"Error: Marker {origin_id} not detected! 👺")
            return {}  
        
        origin = self.transformations[origin_id]
        origin_inv = numphy.linalg.inv(origin)
        tags = {}  # Dictionary to store transformations
        
        bias_x = 0 # Makes the matrix computed closer to the actual value in real life
        bias_y = 0
        bias_z = 0
        
        for tag_id, transformation in self.transformations.items():
            tag_id = int(tag_id)
            if tag_id == origin_id:
                continue
            
            origin_to_tag = numphy.dot(origin_inv, transformation)
            
            # Reorder the transformation matrix elements
            xx = origin_to_tag[1][1]
            xy = -origin_to_tag[0][1]
            xz = origin_to_tag[2][1]
            
            yx = -origin_to_tag[1][0]
            yy = origin_to_tag[0][0]
            yz = -origin_to_tag[2][0]
            
            zx = origin_to_tag[1][2]
            zy = -origin_to_tag[0][2]
            zz = origin_to_tag[2][2]
            
            tx = origin_to_tag[1][3]
            ty = -origin_to_tag[0][3]
            tz = origin_to_tag[2][3]
            
            origin_to_tag = numphy.array([
                [xx, yx, zx, tx+bias_x+offset_x],
                [xy, yy, zy, ty+bias_y+offset_y],
                [xz, yz, zz, tz+bias_z+offset_z],
                [0,  0,  0,  1]
            ]).tolist()


            
            tags[tag_id] = origin_to_tag  # Store result

        return tags
    
    def init_tags(self, *tags: str | int):
        all_tags=[]
        for tag in tags:
            all_tags.append(str(tag))
        return all_tags
    
    def save_to_json(self, *allowed_tags: str | int):

        reader = Jsonreader()
        self.transformations = Aruco.estimate_pose()
        tags = self.transformation_origin_to_tag(0, 0.5, 0.5, 0.5)

        for id in tags.keys():
            id = str(id)

        allowed_tags = self.init_tags(allowed_tags)
        reader.write("camera_readings",tags)        
        data = reader.read("camera_readings")
        
        for key in data.keys():
            if not key in str(allowed_tags):
                reader.pop("camera_readings",key)
                print(f"removed hallucinated tag, id: {key}")    
        
def initalize_system():
    camera = Camera("031422250347", 1280, 720)
    aruco = Aruco()
    coord_sys = CoordinateSystem(marker_length=0.048,origin_id=0)
    return camera, aruco, coord_sys

def main():
    camera, aruco, coord_sys = initalize_system()   

    while True:
        # Update pose estimation
        coord_sys.save_to_json(25)       
        
if __name__ == '__main__':
    main()