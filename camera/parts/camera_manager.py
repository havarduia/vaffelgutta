"""
Camera manager module for discovering, adding, and removing camera instances.
"""

import pyrealsense2 as rs
from typing import Dict, Optional

from camera.parts.camera_instance import CameraInstance


class CameraManager:
    """
    Class for managing multiple camera instances, including discovery, adding, and removing cameras.
    """
    def __init__(self) -> None:
        """Initialize the camera manager."""
        # Discover available RealSense cameras
        self.available_cameras = self.discover_cameras()
        
        if not self.available_cameras:
            raise RuntimeError("No RealSense cameras found.")
            
        # Dictionary to store camera instances
        self.cameras = {}
        self._default_camera = None

    def discover_cameras(self) -> Dict[str, str]:
        """
        Discover available RealSense cameras and return a dictionary of serial numbers.
        
        Returns:
            Dict[str, str]: Dictionary mapping camera serial numbers to camera names
        """
        ctx = rs.context()
        devices = ctx.query_devices()
        camera_dict = {}

        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            camera_dict[serial] = name
            print(f"Found camera {i+1}: {name} (Serial: {serial})")

        return camera_dict

    def add_camera(self, camera_id: Optional[str] = None, name: Optional[str] = None) -> str:
        """
        Add a camera to the vision system.

        Args:
            camera_id: The serial number of the camera to add. If None, uses the first available camera.
            name: A friendly name to use for accessing the camera. If None, uses cam1, cam2, etc.

        Returns:
            The name used to access the camera.
        """
        # If no camera_id provided, use the first available camera not already in use
        if camera_id is None:
            available_ids = set(self.available_cameras.keys()) - set(cam.camera_id for cam in self.cameras.values())
            if not available_ids:
                raise RuntimeError("No available cameras to add.")
            camera_id = list(available_ids)[0]
        elif camera_id not in self.available_cameras:
            raise ValueError(f"Camera with ID {camera_id} not found.")

        # Generate a name if not provided
        if name is None:
            name = f"cam{len(self.cameras) + 1}"

        # Check if name is already in use
        if name in self.cameras:
            raise ValueError(f"Camera name '{name}' is already in use.")

        # Create the camera instance
        camera_instance = CameraInstance(camera_id)
        self.cameras[name] = camera_instance

        # Set as default if this is the first camera
        if not self._default_camera:
            self._default_camera = name

        return name

    def remove_camera(self, name: str) -> None:
        """
        Remove a camera from the vision system.
        
        Args:
            name: The name of the camera to remove
        """
        if name not in self.cameras:
            raise ValueError(f"Camera '{name}' not found.")

        # Clean up resources
        self.cameras[name].cleanup()

        # Remove from cameras dict
        del self.cameras[name]

        # Update default camera if needed
        if self._default_camera == name:
            if self.cameras:
                self._default_camera = next(iter(self.cameras.keys()))
            else:
                self._default_camera = None
                
    def get_camera(self, name: str) -> CameraInstance:
        """
        Get a camera instance by name.
        
        Args:
            name: The name of the camera to get
            
        Returns:
            The camera instance
        """
        if name not in self.cameras:
            raise ValueError(f"Camera '{name}' not found.")
        return self.cameras[name]
        
    def get_default_camera(self) -> Optional[CameraInstance]:
        """
        Get the default camera instance.
        
        Returns:
            The default camera instance, or None if no cameras are added
        """
        if not self._default_camera:
            return None
        return self.cameras[self._default_camera]
        
    def cleanup_all(self) -> None:
        """Clean up resources for all camera instances."""
        for camera in list(self.cameras.values()):
            camera.cleanup()
