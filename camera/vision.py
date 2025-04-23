import cv2
import numpy as np
from typing import Union, List, Dict, Optional
from robot.tools.file_manipulation import Jsonreader
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector


class Vision:
    """Main vision class that integrates camera, marker detection, and coordinate transformation."""

    def __init__(self):
        """Initialize the vision system."""
        self.jsonreader = Jsonreader()
        # Initialize components
        self.camera = RealSense("id2")
        self.aruco = Aruco(self.camera)
        self.coord_sys = CoordinateSystem()
        # Pass the coordinate system to the hand detector so it uses the same instance
        self.hand_detector = HandDetector(self.camera, self.coord_sys)

    def __del__(self):
        """Clean up resources when the object is deleted."""
        pass

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
        draw_cubes: bool = True,
        detect_hands: bool = False,
        detect_gestures: bool = False
    ) -> Optional[np.ndarray]:
        """Process a single frame and update tag positions.

        Args:
            *allowed_tags: Optional list of tag IDs to track
            return_image: Whether to return the annotated image
            detect_hands: Whether to detect and track hands
            detect_gestures: Whether to detect rock-paper-scissors gestures
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

        # Process hand detection or gesture recognition on the same image if requested
        if detect_hands or detect_gestures:
            # Get the color and depth frames from the camera
            color_frame = image
            depth_frame = self.camera.get_depth_frame()
            # If detect_gestures is True, we want to disable 3D coordinate calculation
            # and use gesture recognition instead
            get_3d_coords = not detect_gestures
            result, image = self.hand_detector.process_frame(color_frame, depth_frame, get_3d_coords)

            # If we're detecting gestures, save the result to a JSON file
            if detect_gestures and result:
                try:
                    # Try to read the existing file first
                    existing_data = self.jsonreader.read("hand_gesture")
                except:
                    # If the file doesn't exist, create an empty dictionary
                    existing_data = {}

                # Update the data with the new gesture
                existing_data["gesture"] = result
                self.jsonreader.write("hand_gesture", existing_data)

        # Return image if requested
        if return_image:
            return image
        return None

    def run(
        self,
        *allowed_tags: Union[str, int],
        show_image: bool = True,
        draw_cubes: bool = True,
        detect_hands: bool = False,
        detect_gestures: bool = False
    ) -> None:
        """Continuously run pose estimation, write data, and show video feed.

        Args:
            *allowed_tags: Optional list of tag IDs to track
            show_image: Whether to display the video feed
            detect_hands: Whether to detect and track hands
            detect_gestures: Whether to detect rock-paper-scissors gestures
            draw_cubes: Whether to draw 3D cubes on the markers
        """
        while True:
            img = self.run_once(
                *allowed_tags,
                return_image=True,
                draw_cubes=draw_cubes,
                detect_hands=detect_hands,
                detect_gestures=detect_gestures
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
    # Example usage: Run with gesture detection enabled
    vision.run(show_image=True, draw_cubes=True, detect_gestures=True)