import pyrealsense2 as rs
import numpy as numphy
import cv2

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

    def __del__(self):
        self.stop_streaming()

    def start_streaming(self):
        if not self.isStreaming:
            profile = self.pipeline.start(self.config)
            self.isStreaming = True

            # Get intrinsics from the color stream
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            
            # Store intrinsics
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
        if not self.isStreaming:
            return None
        frame = self.pipeline.wait_for_frames()
        self.color_frame = frame.get_color_frame()
        return numphy.asanyarray(self.color_frame.get_data())

