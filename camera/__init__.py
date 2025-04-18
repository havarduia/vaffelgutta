"""Camera module for detecting markers and tracking objects.

This module provides classes for camera access, marker detection, and coordinate transformation.
"""

from camera.vision import Vision
from camera.base import Camera
from camera.realsense import RealSense
from camera.markers import Aruco
from camera.coordinates import CoordinateSystem

__all__ = ['Vision', 'Camera', 'RealSense', 'Aruco', 'CoordinateSystem']