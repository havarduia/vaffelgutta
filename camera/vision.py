import cv2
import numpy as np
from typing import Union, List, Dict, Optional
from robot.tools.file_manipulation import Jsonreader
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector
from multiprocessing import Process, Manager

def process_aruco(aruco, coord_sys, results, draw_cubes, order, image):
    """
    Process a single frame to detect markers and transform coordinates.
    """
    # Detect markers and estimate poses
    poses, image = aruco.estimate_poses(image, draw_cubes=draw_cubes)
    posematrices = poses.values()
    distances = [np.linalg.norm(np.array([p[3][0], p[3][1], p[3][2]])) for p in posematrices]
    

    # Transform poses to robot coordinate system
    transformed_poses = coord_sys.transform_poses(poses)
    out = [order, transformed_poses, image, distances]
    results.put(out)
    return

class Vision:
    """Main vision class that integrates camera, marker detection, and coordinate transformation."""

    def __init__(self):
        """Initialize the vision system."""
        self.jsonreader = Jsonreader()
        # Initialize components
        self.cameras = dict()
        self.arucos = []
        self.hand_detectors = []
        self.coord_sys = CoordinateSystem()

        for i in range(10):
            try:
                id = f"id{i+1}"
                self.cameras.update({id:RealSense(id)})
            except KeyError: 
                continue
            self.arucos.append(Aruco(self.cameras[id]))
            self.hand_detectors.append(HandDetector(self.cameras[id], self.coord_sys))


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
        manager = Manager()
        aruco_results = manager.Queue() 
        images = [camera.get_color_frame() for camera in self.cameras.values()]
        i = 0
        for image, aruco in zip(images, self.arucos):
            aruco_process = Process(
                    target=process_aruco,
                    args = (aruco, self.coord_sys, aruco_results, draw_cubes, i, image)
                    )
            aruco_processes.append(aruco_process)
            aruco_process.start()
            i+=1

        handimg = None
        # Process hand detection or gesture recognition
        if detect_hands:
            assert not detect_gestures, "detect_hands and detect_gestures should never run at the same time"
            try:
                camera = self.cameras["id2"]
                image, depth = camera.get_aligned_frames()
                hand_detector = self.hand_detectors[0]
                result, image = self._process_hand(image, depth, False, hand_detector)
                handimg = image

                self.jsonreader.clear("hand_position")
                if result is not None:
                    self.jsonreader.write("hand_position", {"position" : str(result)})
                    pass
            except KeyError:
                print("camera not connected!")

        if detect_gestures:
            assert not detect_hands, "detect_hands and detect_gestures should never run at the same time"
            try:
                camera = self.cameras["id1"]
                image, depth = camera.get_aligned_frames()
                hand_detector = self.hand_detectors[0]
                result, image = self._process_hand(image, depth, True, hand_detector)
                handimg = image
                
                self.jsonreader.clear("hand_gesture")
                if result is not None:
                    self.jsonreader.write("hand_gesture", {"gesture" : str(result)})
            except KeyError:
                print("camera not connected!")

        # save tags to file 
        for p in aruco_processes:
            p.join()
        taglist = [0]*len(self.arucos) 
        imglist = [0]*len(self.arucos) 
        distancelist = [0]*len(self.arucos)
        while not aruco_results.empty():
            results = aruco_results.get()
            (order, transformed_poses, image, distances) =results
            taglist[order] = transformed_poses
            imglist[order] = image
            distancelist[order] = distances
        
        tags = self._shortest_tags(taglist,distancelist)
        tags = self._sort_tags(tags, allowed_tags)
        # Save to JSON
        self.jsonreader.write("camera_readings", tags)

        if handimg is not None: imglist.append(handimg)
        # Return image if requested
        return imglist if return_image else None

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
                    cv2.imshow(f"Pose Estimation Feed {i+1}", img)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC to break
                cv2.destroyAllWindows()
                break


# =====================================================
# Example Usage
# =====================================================
if __name__ == "__main__":
    vision = Vision()
    # Example usage: Run with gesture detection enabled
    vision.run(show_image=True, draw_cubes=True, detect_hands=True)
