"""
Camera parts module.

This module contains components for camera functionality.
"""

from camera.parts.base import Camera
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector
from camera.parts.marker_thread import MarkerDetectionThread
from camera.parts.camera_manager import CameraManager

__all__ = [
    'Camera',
    'RealSense',
    'Aruco',
    'CoordinateSystem',
    'HandDetector',
    'MarkerDetectionThread',
    'CameraManager'
]
