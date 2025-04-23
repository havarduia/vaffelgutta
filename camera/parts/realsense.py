"""
RealSense camera implementation.
"""

import pyrealsense2 as rs
import numpy as np
from camera.parts.base import Camera
from camera.config.configloader import ConfigLoader


class RealSense(Camera):

    def __init__(self, camera_id=None):
        """
        Initialize the RealSense camera.
        """
        super().__init__()
        self.config_loader = ConfigLoader()
<<<<<<< HEAD

        if camera_id is None:
            raise TypeError("No camera specified!")

        # Get the actual serial number from the config using the camera_id as the key
        self.serial_number = self.config_loader.get(camera_id)

=======
        if camera_id is None:
            raise TypeError("No camera specified!")
        
        self.camera_id = self.config_loader.get(camera_id)
>>>>>>> 6602363920169fb8c41f10611880be355fc4a790
        # Camera setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams
        resolution = self.config_loader.get("resolution")
        resolution_d = self.config_loader.get("resolution_d")
        fps = self.config_loader.get("fps")
        fps_d = self.config_loader.get("fps_d")

        self.camera_id = camera_id
        print(f"Using camera ID: '{self.camera_id}' with serial number: '{self.serial_number}'")
        self.config.enable_device(self.serial_number)

        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps
        )
        self.config.enable_stream(
            rs.stream.depth, resolution_d[0], resolution_d[1], rs.format.z16, fps_d
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

    def get_aligned_frames(self):
        """Get an aligned color and depth frame from the camera."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = rs.align(rs.stream.color).process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return np.asanyarray(color_frame.get_data()), np.asanyarray(
            depth_frame.get_data()
        )

    def get_depth_scale(self):
        """Get the depth scale of the camera."""
        return (
            self.pipeline.get_active_profile()
            .get_device()
            .first_depth_sensor()
            .get_depth_scale()
        )

    def get_rs_intrinsics(self):
        """Get the pyrealsense2 intrinsics object for the color stream."""
        if not self.isStreaming or not hasattr(self, "intrinsics"):
            # Or handle appropriately, maybe try starting stream again or raise error
            print("Warning: Stream not active or intrinsics not available.")
            return None
        return self.intrinsics  # Return the stored rs.intrinsics object

