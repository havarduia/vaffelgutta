import pyrealsense2 as rs
import numpy as np
from camera.Config.misc import print_blue, print_error, ConfigLoader
import os

class Camera:
    def __init__(self, config_loader):
        self.isStreaming = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Automatically find the first available RealSense device
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            print_error("No RealSense camera detected.")
            raise RuntimeError("No RealSense camera found.")
        
        camera_id = devices[0].get_info(rs.camera_info.serial_number)
        print_blue(f"Using RealSense camera with serial: {camera_id}")

        resolution = config_loader.get("resolution") 
        fps = config_loader.get("fps") 

        self.config.enable_device(camera_id)
        self.config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)
        
        self.color_frame = None
        self.intrinsics = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.start_streaming()

    def __del__(self):
        self.stop_streaming()

    def start_streaming(self):
        if not self.isStreaming:
            try:
                profile = self.pipeline.start(self.config)
                color_sensor = profile.get_device().query_sensors()[1]
                color_sensor.set_option(rs.option.sharpness, 100) 
            except Exception as e:
                print_error(f"Error starting pipeline: {e}")
                raise e
            
            self.isStreaming = True
            self.data = np.load(os.path.expanduser('~/git/vaffelgutta/camera/camera_calibration.npz'))
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

    def stop_streaming(self):
        if self.isStreaming:
            self.pipeline.stop()
            self.isStreaming = False

    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        image = np.asanyarray(color_frame.get_data())
        return image

    def get_calibration(self):
        """Return the camera matrix and distortion coefficients."""
        return self.camera_matrix, self.dist_coeffs
