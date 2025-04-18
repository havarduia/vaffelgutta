"""
RealSense camera implementation.
"""

import pyrealsense2 as rs
import numpy as np
from camera.base import Camera
from camera.config.configloader import ConfigLoader


class RealSense(Camera):
    """
    Implementation of the Camera interface for Intel RealSense cameras.
    """

    def __init__(self):
        """Initialize the RealSense camera."""
        super().__init__()
        self.config_loader = ConfigLoader()

        # Camera setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Find the first available RealSense camera
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("No RealSense camera found.")

        self.camera_id = devices[0].get_info(rs.camera_info.serial_number)
        # Camera initialized with serial number

        # Configure streams
        resolution = self.config_loader.get("resolution")
        resolution_d = self.config_loader.get("resolution_d")
        fps = self.config_loader.get("fps")

        self.config.enable_device(self.camera_id)
        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps
        )
        self.config.enable_stream(
            rs.stream.depth, resolution_d[0], resolution_d[1], rs.format.z16, fps
        )

        # Start streaming
        self._start_streaming()

    def _start_streaming(self):
        """Start the camera stream."""
        if not self.isStreaming:
            try:
                profile = self.pipeline.start(self.config)
                # Set camera options
                color_sensor = profile.get_device().query_sensors()[1]
                color_sensor.set_option(rs.option.sharpness, 100)

                # Get camera intrinsics
                intrinsics = (
                    profile.get_stream(rs.stream.color)
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

                # Set calibration parameters
                self.intrinsics = intrinsics
                self.camera_matrix = np.array(
                    [
                        [intrinsics.fx, 0, intrinsics.ppx],
                        [0, intrinsics.fy, intrinsics.ppy],
                        [0, 0, 1],
                    ]
                )
                self.dist_coeffs = np.array(intrinsics.coeffs[:5])

                self.isStreaming = True
            except Exception as e:
                # Log error but continue
                print(f"Camera error: {e}")
                raise e

    def _stop_streaming(self):
        """Stop the camera stream."""
        if self.isStreaming:
            self.pipeline.stop()
            self.isStreaming = False

    def get_color_frame(self):
        """Get a color frame from the camera."""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def get_depth_frame(self):
        """Get a depth frame from the camera."""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        return np.asanyarray(depth_frame.get_data())
