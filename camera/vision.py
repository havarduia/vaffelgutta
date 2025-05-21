import cv2
import numpy as np
from typing import Union, List, Dict, Optional
from robot.tools.file_manipulation import Jsonreader
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector

def process_aruco(aruco, coord_sys, draw_cubes, image):
    """
    Process a single frame to detect markers and transform coordinates.
    """
    # Detect markers and estimate poses
    poses, image = aruco.estimate_poses(image, draw_cubes=draw_cubes)
    posematrices = poses.values()
    distances = [np.linalg.norm(np.array([p[3][0], p[3][1], p[3][2]])) for p in posematrices]

    # Transform poses to robot coordinate system
    transformed_poses = coord_sys.transform_poses(poses)
    out = [ transformed_poses, image, distances]
    return out

class Vision:
    """Main vision class that integrates camera, marker detection, and coordinate transformation."""

    def __init__(self):
        """Initialize the vision system."""
        self.jsonreader = Jsonreader()
        # Initialize components
        self.cameras = dict()
        self.arucos = dict()
        self.hand_detectors = dict() 
        self.coord_sys = CoordinateSystem()

        for i in range(10):
            try:
                id = f"id{i+1}"
                self.cameras.update({id:RealSense(id)})
            except KeyError: 
                continue
            self.arucos.update({id : Aruco(self.cameras[id])})
            self.hand_detectors.update({id: HandDetector(self.cameras[id], self.coord_sys)})

    def __del__(self):
        """Clean up resources when the object is deleted."""
        pass


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


    def _shortest_tags(self, taglist, distancelist) -> dict:
        # sort tags by distance
        tags = dict() 
        distances = dict()
        for temptags, tempdistances in zip(taglist, distancelist):
            for (tagid, tagvalue), distance in zip(temptags.items(), tempdistances):
                #if tag does not exist yet, write it without question
                if tagid not in tags or distances[tagid] < distance:
                    tags.update({tagid:tagvalue})
                    distances.update({tagid:distance})
        return tags

    def _sort_tags(self, tags, allowed_tags) -> dict:
        # Convert allowed_tags to strings
        allowed_tag_strings = [str(tag) for tag in allowed_tags]

        # Filter tags if needed
        if allowed_tag_strings:
            tags = self._filter_tags(tags, allowed_tag_strings)
        return tags

    def _process_hand(self, image, depth, detect_gestures, hand_detector):
            # Get the color and depth frames from the camera
            color_frame = image
            depth_frame = depth
            # If detect_gestures is True, we want to disable 3D coordinate calculation
            # and use gesture recognition instead
            get_3d_coords = not detect_gestures
            return hand_detector.process_frame(color_frame, depth_frame, get_3d_coords)


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
        # aruco detection:
        images = dict()
        depths = dict()
        for id, camera in self.cameras.items():
            image, depth = camera.get_aligned_frames()
            images.update({id: image}) 
            depths.update({id: depth})

        results = dict()
        for id, image, aruco in zip(images.keys(), images.values(), self.arucos.values()):
            results.update({id: process_aruco(aruco, self.coord_sys, draw_cubes, image)})
        # save tags to file 
        taglist = dict() 
        distancelist = dict() 
        for id,result in results.items():
            (transformed_poses, image, distances) =result
            taglist[id] = transformed_poses
            distancelist[id] = distances
            images[id] = image
        
        tags = self._shortest_tags(taglist.values(),distancelist.values())
        tags = self._sort_tags(tags, allowed_tags)
        # Save to JSON
        self.jsonreader.write("camera_readings", tags)

        # Process hand detection or gesture recognition
        if detect_gestures:
            try:
                image = images["id1"]
                depth = depths["id1"]
                hand_detector = self.hand_detectors["id1"]
                result, image = self._process_hand(image, depth, True, hand_detector)
                images.update({"id1": image})
            except (KeyError, IndexError):
                print("main camera not connected!")
                raise RuntimeError
            self.jsonreader.clear("hand_gesture")
            if result is not None:
                self.jsonreader.write("hand_gesture", {"gesture" : str(result)})

        if detect_hands:
            try:
                image = images["id2"]
                depth = depths["id2"]
                hand_detector = self.hand_detectors["id2"]
                result, image = self._process_hand(image, depth, False, hand_detector)
                images.update({"id2": image})
            except KeyError:
                print("support camera not connected!")
                raise RuntimeError
            self.jsonreader.clear("hand_position")
            if result is not None:
                self.jsonreader.write("hand_position", {"position" : str(result)})
                pass

    


        # Return image if requested
        return images.values() if return_image else None

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
            imglist = self.run_once(
                *allowed_tags,
                return_image=True,
                draw_cubes=draw_cubes,
                detect_hands=detect_hands,
                detect_gestures=detect_gestures
            )
            if show_image and imglist is not None:
                for i, img in enumerate(imglist):
                    cv2.namedWindow(f"Pose Estimation Feed {i+1}", cv2.WINDOW_NORMAL)
                    cv2.imshow(f"Pose Estimation Feed {i+1}", img)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC to break
                cv2.destroyAllWindows()
                break


# =====================================================
# Example Usage
# =====================================================
if __name__ == "__main__":
    vision = Vision()
    vision.run(show_image=True, draw_cubes=True, detect_hands=True)


    # add a maleman to run vision.runonce() on a message (you choose the name) (msg field is None (i think))

>>>>>>> integrate_screen
