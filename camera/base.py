"""
Base camera module providing abstract camera functionality.
"""

from abc import ABC, abstractmethod
import numpy as np


class Camera(ABC):
    """
    Abstract base class for camera implementations.

    This class defines the interface that all camera implementations must follow.
    """

    def __init__(self):
        """Initialize the camera."""
        self.isStreaming = False
        self.camera_matrix = None
        self.dist_coeffs = None

    @abstractmethod
    def _start_streaming(self):
        """Start the camera stream."""
        pass

    @abstractmethod
    def _stop_streaming(self):
        """Stop the camera stream."""
        pass

    @abstractmethod
    def get_color_frame(self):
        """Get a color frame from the camera."""
        pass

    @abstractmethod
    def get_depth_frame(self):
        """Get a depth frame from the camera."""
        pass

    def get_calibration(self):
        """Return the camera matrix and distortion coefficients."""
        return self.camera_matrix, self.dist_coeffs

    def __del__(self):
        """Clean up resources when the object is deleted."""
        self._stop_streaming()
