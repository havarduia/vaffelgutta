import cv2
from typing import Optional

from camera.parts.camera_instance import CameraInstance
from camera.parts.camera_manager import CameraManager


class Vision:
    """Main vision class that manages multiple camera instances."""
    def __init__(self) -> None:
        # Initialize the camera manager
        self.camera_manager = CameraManager()

        # For backward compatibility
        self.available_cameras = self.camera_manager.available_cameras

    def add_camera(self, camera_id: Optional[str] = None, name: Optional[str] = None) -> str:
        """Add a camera to the vision system.

        Args:
            camera_id: The serial number of the camera to add. If None, uses the first available camera.
            name: A friendly name to use for accessing the camera. If None, uses cam1, cam2, etc.

        Returns:
            The name used to access the camera.
        """
        return self.camera_manager.add_camera(camera_id, name)

    def remove_camera(self, name: str) -> None:
        """Remove a camera from the vision system."""
        self.camera_manager.remove_camera(name)

    def __getattr__(self, name: str) -> CameraInstance:
        """Allow accessing cameras as attributes (e.g., vision.cam1)."""
        try:
            return self.camera_manager.get_camera(name)
        except ValueError:
            raise AttributeError(f"'Vision' object has no attribute '{name}'")

    def run_all(self, *args, **kwargs) -> None:
        """Run all cameras simultaneously."""
        if not self.camera_manager.cameras:
            raise RuntimeError("No cameras added to vision system.")

        # Convert show_image to return_image for run_once calls
        run_once_kwargs = kwargs.copy()
        if 'show_image' in run_once_kwargs:
            run_once_kwargs['return_image'] = run_once_kwargs.pop('show_image')
        else:
            run_once_kwargs['return_image'] = True

        try:
            # Create windows for each camera
            for name, camera in self.camera_manager.cameras.items():
                # Start each camera in its own window
                window_name = f"Camera {name}"
                # Don't pass window_name to run_once
                img = camera.run_once(*args, **run_once_kwargs)
                if img is not None:
                    cv2.imshow(window_name, img)

            # Main loop
            while True:
                for name, camera in self.camera_manager.cameras.items():
                    img = camera.run_once(*args, **run_once_kwargs)
                    if img is not None:
                        cv2.imshow(f"Camera {name}", img)

                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            # Clean up all cameras
            self.camera_manager.cleanup_all()
            cv2.destroyAllWindows()

    def __del__(self):
        """Clean up resources when the object is deleted."""
        if hasattr(self, 'camera_manager'):
            self.camera_manager.cleanup_all()


if __name__ == "__main__":
    vision = Vision()

    # Add cameras - will use the first available camera if no ID is specified
    cam1_name = vision.add_camera(name="cam1")
    cam2_name = vision.add_camera(name="cam2")
    
    vision.cam1.run(show_image=True, detect_markers=True, detect_hands=True)
