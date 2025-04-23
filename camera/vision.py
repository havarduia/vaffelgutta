import cv2
import numpy as np
from typing import Union, List, Dict, Optional
from robot.tools.file_manipulation import Jsonreader
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector
from multiprocessing import Queue, Process 


def process_aruco(aruco, coord_sys, results: Queue, draw_cubes, order):
    """Process a single frame to detect markers and transform coordinates.

    Args:
        draw_cubes: Whether to draw 3D cubes on the markers

    Returns:
        tuple: (transformed_poses, annotated_image)
    """
    # Detect markers and estimate poses
    poses, image = aruco.estimate_poses(draw_cubes=draw_cubes)
    posematrices = poses.values()
    distances = [np.linalg.norm(p[3][0], p[3][1],p[3][2]) for p in posematrices]
    # Transform poses to robot coordinate system
    transformed_poses = coord_sys.transform_poses(poses)
    results.put([order, transformed_poses, image, distances])
    return

class Vision:
    """Main vision class that integrates camera, marker detection, and coordinate transformation."""

    def __init__(self):
        """Initialize the vision system."""
        self.jsonreader = Jsonreader()
        # Initialize components
        self.camera_ids = []
        self.cameras = []
        self.arucos = []
        self.hand_detectors = []
        self.coord_sys = CoordinateSystem()

        for i in range(10):
            try:
                id = f"id{i+1}"
                self.cameras.append(RealSense(id))
            except KeyError: 
                break
            self.camera_ids.append(i)
            self.arucos.append(Aruco(self.cameras[i]))
            self.hand_detectors.append(HandDetector(self.cameras[i], self.coord_sys))
        print(self.cameras)


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
        aruco_processes = list()
        aruco_results = Queue() 
        for i, aruco in enumerate(self.arucos):
            aruco_process = Process(
                    target=process_aruco,
                    args = (aruco, self.coord_sys, draw_cubes, aruco_results,i)
                    )
            aruco_processes.append(aruco_process)
            aruco_process.start()

        # Process hand detection or gesture recognition
        if detect_hands:
            assert not detect_gestures, "detect_hands and detect_gestures should never run at the same time"
            camera = self.cameras[0]
            image, depth = camera.get_aligned_frame()
            hand_detector = self.hand_detectors[0]
            result, image = self._process_hand(image, depth, False, hand_detector)

            self.jsonreader.clear("hand_position")
            if result is not None:
                self.jsonreader.write("hand_position", {"position" : result})

        if detect_gestures:
            assert not detect_hands, "detect_hands and detect_gestures should never run at the same time"
            camera = self.cameras[1]
            image, depth = camera.get_aligned_frame()
            hand_detector = self.hand_detectors[1]
            result, image = self._process_hand(image, depth, True, hand_detector)
            
            self.jsonreader.clear("hand_gesture")
            if result is not None:
                self.jsonreader.write("hand_gesture", {"gesture" : result})

        # save tags to file 
        for p in aruco_processes:
            p.join()
        taglist = list()
        imglist = list()
        distancelist = list()
        while not aruco_results.empty():
            (order, transformed_poses, image, distances) = aruco_results.get()
            taglist[order] = transformed_poses
            imglist[order] = image
            distancelist[order] = distances

        tags = self._shortest_tags(taglist,distancelist)
        tags = self._sort_tags(tags, allowed_tags)
        # Save to JSON
        self.jsonreader.write("camera_readings", tags)

        # Return image if requested
        return imglist[0] if return_image else None

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
