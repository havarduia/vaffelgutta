import pyrealsense2 as rs
import numpy as numphy
numphy.set_printoptions(suppress=True, precision=4)
from camera.misc import print_blue, print_error, ConfigLoader


class Camera:
    def __init__(self, config_loader):

        self.isStreaming = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        camera_id = config_loader.get("camera_id")
        resolution = config_loader.get("resolution")
        fps = config_loader.get("fps")

        self.config.enable_device(camera_id)
        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps
        )
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
            except Exception as e:
                print_error(f"Error starting pipeline: {e}")
                raise e
            self.isStreaming = True

            # Get intrinsics from the color stream.
            intrinsics = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
            self.intrinsics = intrinsics
            self.camera_matrix = numphy.array(
                [
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1],
                ]
            )
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
