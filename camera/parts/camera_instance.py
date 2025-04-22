"""
Camera instance module that encapsulates a single camera with all its components.
"""

import cv2
import numpy as np
from typing import Union, Optional, List, Dict, Any, Tuple

from robot.tools.file_manipulation import Jsonreader
from camera.parts.realsense import RealSense
from camera.parts.markers import Aruco
from camera.parts.coordinates import CoordinateSystem
from camera.parts.hand_detection import HandDetector
from camera.parts.marker_thread import MarkerDetectionThread


class CameraInstance:
    """A class representing a single camera instance with all its associated components."""
    def __init__(self, camera_id: str = None) -> None:
        self.jsonreader = Jsonreader()
        self.camera = RealSense(camera_id)
        self.aruco = Aruco(self.camera)
        self.coord_sys = CoordinateSystem()
        self.hand_detector = HandDetector(self.camera, self.coord_sys)

        # Initialize marker detection thread but don't start it yet
        self.marker_thread = MarkerDetectionThread(
            camera=self.camera,
            aruco=self.aruco,
            coord_sys=self.coord_sys,
            jsonreader=self.jsonreader
        )
        self.marker_thread_running = False

        # Store the camera ID for reference
        self.camera_id = camera_id or self.camera.camera_id

    def _acquire_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        # Use aligned frames for better performance and accuracy
        color, depth = self.camera.get_aligned_frames()
        if color is None:
            raise RuntimeError("No aligned color frame available")
        return color, depth

    def _start_marker_thread_if_needed(self) -> None:
        """Start the marker detection thread if it's not already running."""
        if not self.marker_thread_running:
            self.marker_thread.start()
            self.marker_thread_running = True
            print(f"Marker detection thread started for camera {self.camera_id}")

    def _stop_marker_thread_if_running(self) -> None:
        """Stop the marker detection thread if it's running."""
        if self.marker_thread_running and hasattr(self, 'marker_thread'):
            self.marker_thread.stop()
            self.marker_thread.join(timeout=1.0)
            self.marker_thread_running = False
            print(f"Marker detection thread stopped for camera {self.camera_id}")

    def _detect_markers(
        self,
        color_frame: np.ndarray,
        allowed_tags: Tuple[Union[int, str], ...],
        draw_cubes: bool
    ) -> Tuple[np.ndarray, Dict[int, Any]]:
        # Make sure the marker thread is running
        self._start_marker_thread_if_needed()

        # Update thread parameters
        self.marker_thread.set_draw_cubes(draw_cubes)
        self.marker_thread.set_allowed_tags(allowed_tags)

        # Get the latest results from the thread
        results = self.marker_thread.get_latest_results()

        # Use the latest image if available, otherwise use the provided color frame
        img = results['image'] if results['image'] is not None else color_frame

        # Get the transformed poses
        transformed = results['transformed']

        # Display FPS and processing time on the image
        if img is not None:
            cv2.putText(
                img,
                f"FPS: {results['fps']:.1f} | Processing: {results['processing_time']*1000:.1f}ms",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        return img, transformed

    def _detect_hands(
        self,
        image: np.ndarray,
        depth_frame: np.ndarray,
        gesture_off: bool
    ) -> Tuple[np.ndarray, List[Any]]:
        results, hand_img = self.hand_detector.process_frame(
            image, depth_frame, get_3d_coords=gesture_off
        )
        img = hand_img if hand_img is not None else image

        if results is not None and isinstance(results, np.ndarray) and results.shape == (4, 4):
            # Clear the file first to remove any old data
            self.jsonreader.clear("hand_position")

            # Format the matrix as requested
            hand_data = {
                "hand": results.tolist()
            }
            self.jsonreader.write("hand_position", hand_data)

        return img, results

    def run_once(
        self,
        *allowed_tags: Union[int, str],
        return_image: bool = False,
        draw_cubes: bool = True,
        detect_markers: bool = True,
        detect_hands: bool = False,
        gesture_off: bool = True
    ) -> Optional[np.ndarray]:
        # Stop marker thread if not needed
        if not detect_markers and self.marker_thread_running:
            self._stop_marker_thread_if_running()

        # if no processing requested, optionally just return the current frame
        if not detect_markers and not detect_hands:
            if return_image:
                return self.camera.get_color_frame()
            return None

        color_frame, depth_frame = self._acquire_frames()
        processed = color_frame.copy()

        if detect_markers:
            processed, _ = self._detect_markers(color_frame, allowed_tags, draw_cubes)

        if detect_hands:
            processed, _ = self._detect_hands(processed, depth_frame, gesture_off)

        return processed if return_image else None

    def run(
        self,
        *allowed_tags: Union[int, str],
        show_image: bool = True,
        draw_cubes: bool = True,
        detect_hands: bool = False,
        detect_markers: bool = True,
        gesture_off: bool = True,
        window_name: str = None
    ) -> None:
        try:
            window_title = window_name or f"Camera {self.camera_id}"
            while True:
                img = self.run_once(
                    *allowed_tags,
                    return_image=show_image,
                    draw_cubes=draw_cubes,
                    detect_markers=detect_markers,
                    detect_hands=detect_hands,
                    gesture_off=gesture_off
                )

                if img is not None and show_image:
                    cv2.imshow(window_title, img)

                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            # Stop the marker detection thread
            self._stop_marker_thread_if_running()
            cv2.destroyWindow(window_title)

    def cleanup(self):
        """Clean up resources for this camera instance."""
        self._stop_marker_thread_if_running()
