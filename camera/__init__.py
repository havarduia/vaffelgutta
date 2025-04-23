"""Camera module for detecting markers and tracking objects.

This module provides classes for camera access, marker detection, and coordinate transformation.
"""

from camera.vision import Vision
from camera.parts.base import Camera
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.camera_manager import CameraManager

__all__ = ['Vision', 'Camera', 'RealSense', 'Aruco', 'CoordinateSystem', 'CameraManager']