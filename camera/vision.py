import cv2
import numpy as np
from typing import Union, List, Dict, Optional
from robot.tools.file_manipulation import Jsonreader
from camera.realsense import RealSense
from camera.markers import Aruco
from camera.coordinates import CoordinateSystem
from camera.ai.hand_detection import HandDetector


class Vision:
    """Main vision class that integrates camera, marker detection, and coordinate transformation."""

    def __init__(self):
        """Initialize the vision system."""
        self.jsonreader = Jsonreader()
        # Initialize components
        self.camera = RealSense()
        self.aruco = Aruco(self.camera)
        self.coord_sys = CoordinateSystem()
        # Pass the coordinate system to the hand detector so it uses the same instance
        self.hand_detector = HandDetector(self.camera, self.coord_sys)

    def __del__(self):
        """Clean up resources when the object is deleted."""
        pass

    def set_hand_bias(self, x=None, y=None, z=None):
        """Set bias values for hand position adjustment and save to config file.

        Args:
            x: Bias in x direction (forward/backward)
            y: Bias in y direction (left/right)
            z: Bias in z direction (up/down)
        """
        self.hand_detector.set_hand_bias(x, y, z)

    def reload_hand_bias(self):
        """Reload hand bias values from the config file."""
        self.hand_detector.load_hand_bias_from_config()

    def _get_calibration(self):
        """Return the camera matrix and distortion coefficients."""
        return self.camera.get_calibration()

    def _process_frame(self, draw_cubes=True):
        """Process a single frame to detect markers and transform coordinates.

        Args:
            draw_cubes: Whether to draw 3D cubes on the markers

        Returns:
            tuple: (transformed_poses, annotated_image)
        """
        # Detect markers and estimate poses
        poses, image = self.aruco.estimate_poses(draw_cubes=draw_cubes)

        # Transform poses to robot coordinate system
        transformed_poses = self.coord_sys.transform_poses(poses)

        return transformed_poses, image

    def _filter_tags(self, tags: Dict, allowed_tags: List[str]) -> Dict:
        """Filter tags based on allowed tag IDs.

        Args:
            tags: Dictionary of tag poses
            allowed_tags: List of allowed tag IDs as strings

        Returns:
            dict: Filtered dictionary of tag poses
        """
        if not allowed_tags:
            return tags

        filtered_tags = {}
        for tag_id, pose in tags.items():
            if str(tag_id) in allowed_tags:
                filtered_tags[tag_id] = pose
        return filtered_tags

    def run_once(
        self,
        *allowed_tags: Union[str, int],
        return_image: bool = False,
        detect_hands: bool = False,
        draw_cubes: bool = True
    ) -> Optional[np.ndarray]:
        """Process a single frame and update tag positions.

        Args:
            *allowed_tags: Optional list of tag IDs to track
            return_image: Whether to return the annotated image
            detect_hands: Whether to detect and track hands
            draw_cubes: Whether to draw 3D cubes on the markers

        Returns:
            Optional[np.ndarray]: Annotated image if return_image is True, otherwise None
        """
        # Convert allowed_tags to strings
        allowed_tag_strings = [str(tag) for tag in allowed_tags]

        # Process frame
        tags, image = self._process_frame(draw_cubes=draw_cubes)

        # Filter tags if needed
        if allowed_tag_strings:
            tags = self._filter_tags(tags, allowed_tag_strings)

        # Save to JSON
        self.jsonreader.write("camera_readings", tags)

        # Detect hands if requested
        if detect_hands and return_image and image is not None:
            # Check if origin marker is visible (no need to print)

            # Pass the marker detection image and the latest tags to the hand detector
            _, hand_image = self.hand_detector.start(
                "hand_position", input_image=image, raw_tags=tags
            )
            # Hand detection status is shown by the HandDetector
            if hand_image is not None:
                # Use the combined image with both markers and hand landmarks
                image = hand_image

        # Return image if requested
        if return_image:
            return image
        return None

    def run(
        self,
        *allowed_tags: Union[str, int],
        show_image: bool = True,
        detect_hands: bool = False,
        draw_cubes: bool = True
    ) -> None:
        """Continuously run pose estimation, write data, and show video feed.

        Args:
            *allowed_tags: Optional list of tag IDs to track
            show_image: Whether to display the video feed
            detect_hands: Whether to detect and track hands
            draw_cubes: Whether to draw 3D cubes on the markers
        """
        while True:
            img = self.run_once(
                *allowed_tags, return_image=True, detect_hands=detect_hands, draw_cubes=draw_cubes
            )

            if show_image and img is not None:
                cv2.imshow("Pose Estimation Feed", img)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC to break
                cv2.destroyAllWindows()
                break


# =====================================================
# Example Usage
# =====================================================
if __name__ == "__main__":
    vision = Vision()
    vision.run(detect_hands=False, show_image=True, draw_cubes=True)
