# camera/parts/hand_detection.py

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import math
from camera.parts.realsense import RealSense  # Adjust import path if needed


class HandDetector:
    """
    A simple class to detect hands using MediaPipe and a RealSense camera.
    Can provide either 3D coordinates of the wrist or Rock/Paper/Scissors gesture recognition.
    """

    def __init__(
        self, camera, coord_sys=None, max_hands=1, detection_con=0.7, track_con=0.5
    ):

        self.camera = camera
        self.coord_sys = coord_sys  # Coordinate system for transformations
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=detection_con,
            min_tracking_confidence=track_con,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.results = None

        # Define landmark IDs (useful for clarity)
        self.LM_WRIST = 0
        self.LM_THUMB_TIP = 4
        self.LM_INDEX_TIP = 8
        self.LM_MIDDLE_TIP = 12
        self.LM_RING_TIP = 16
        self.LM_PINKY_TIP = 20
        self.LM_INDEX_PIP = 6
        self.LM_MIDDLE_PIP = 10
        self.LM_RING_PIP = 14
        self.LM_PINKY_PIP = 18
        self.LM_INDEX_MCP = 5
        self.LM_MIDDLE_MCP = 9  # Often used as a palm reference
        self.LM_PINKY_MCP = 17

        print("HandDetector initialized.")

    # --- Helper Methods (Keep _get_landmark_pixel_coords, _get_depth_at_pixel, _deproject_pixel_to_point_safe) ---
    def _get_landmark_pixel_coords(self, hand_landmarks, landmark_id, img_shape):
        """Gets the pixel coordinates (x, y) for a specific landmark."""
        if hand_landmarks and landmark_id < len(hand_landmarks.landmark):
            landmark = hand_landmarks.landmark[landmark_id]
            height, width, _ = img_shape
            cx, cy = int(landmark.x * width), int(landmark.y * height)
            cx = max(0, min(cx, width - 1))
            cy = max(0, min(cy, height - 1))
            return cx, cy
        return None

    def _get_depth_at_pixel(self, depth_frame, x, y):
        """Safely gets depth value at pixel (x, y)."""
        if depth_frame is None or not (
            0 <= y < depth_frame.shape[0] and 0 <= x < depth_frame.shape[1]
        ):
            return 0.0
        return depth_frame[y, x]

    def _deproject_pixel_to_point_safe(self, intrinsics, pixel, depth):
        """Safely deprojects pixel to point, handling potential errors."""
        if intrinsics is None or depth <= 0:
            return None
        try:
            depth_scale = self.camera.get_depth_scale()
            depth_in_meters = depth * depth_scale
            point = rs.rs2_deproject_pixel_to_point(
                intrinsics, [pixel[0], pixel[1]], depth_in_meters
            )
            return np.array(point)
        except Exception as e:
            print(f"Error during deprojection: {e}")
            return None

    # --- Gesture Recognition Logic ---
    def _recognize_gesture(self, hand_landmarks, img_shape):
        """Recognizes Rock, Paper, or Scissors."""
        landmarks = hand_landmarks.landmark
        if not landmarks:
            return "Unknown"

        try:
            # Get pixel coordinates for relevant landmarks
            coords = {}
            for lm_id in [
                self.LM_WRIST,
                self.LM_THUMB_TIP,
                self.LM_INDEX_PIP,
                self.LM_INDEX_TIP,
                self.LM_MIDDLE_PIP,
                self.LM_MIDDLE_TIP,
                self.LM_RING_PIP,
                self.LM_RING_TIP,
                self.LM_PINKY_PIP,
                self.LM_PINKY_TIP,
                self.LM_MIDDLE_MCP,
            ]:  # Using middle MCP as a palm ref point
                coords[lm_id] = self._get_landmark_pixel_coords(
                    hand_landmarks, lm_id, img_shape
                )
                if coords[lm_id] is None:
                    # print(f"Warning: Failed to get coords for landmark {lm_id}")
                    return "Error"  # Need all landmarks for reliable detection

            # Calculate a reference distance - e.g., wrist to middle finger MCP
            # This helps normalize for hand size variation to some extent
            ref_dist = math.dist(coords[self.LM_WRIST], coords[self.LM_MIDDLE_MCP])
            if ref_dist < 1:  # Avoid division by zero / tiny distances
                ref_dist = 1

            # --- Heuristics for Finger Straightness ---
            # Compare fingertip distance to PIP joint distance from wrist/MCP
            # A straight finger's tip is much further than its PIP joint
            def is_finger_straight(tip_id, pip_id):
                tip_dist = math.dist(coords[self.LM_WRIST], coords[tip_id])
                pip_dist = math.dist(coords[self.LM_WRIST], coords[pip_id])
                # Threshold: Tip should be significantly further than PIP
                # Tip distance should also be large relative to reference distance
                return (
                    tip_dist > pip_dist * 1.3 and tip_dist > ref_dist * 0.7
                )  # Tunable thresholds

            index_straight = is_finger_straight(self.LM_INDEX_TIP, self.LM_INDEX_PIP)
            middle_straight = is_finger_straight(self.LM_MIDDLE_TIP, self.LM_MIDDLE_PIP)
            ring_straight = is_finger_straight(self.LM_RING_TIP, self.LM_RING_PIP)
            pinky_straight = is_finger_straight(self.LM_PINKY_TIP, self.LM_PINKY_PIP)

            # --- Thumb Straightness (Simpler Check) ---
            # Check distance of thumb tip from wrist relative to reference distance
            thumb_tip_dist = math.dist(coords[self.LM_WRIST], coords[self.LM_THUMB_TIP])
            thumb_straight = thumb_tip_dist > ref_dist * 0.8  # Tunable

            # --- Gesture Classification ---
            if (
                index_straight
                and middle_straight
                and ring_straight
                and pinky_straight
                and thumb_straight
            ):
                return "Paper"
            elif (
                index_straight
                and middle_straight
                and not ring_straight
                and not pinky_straight
            ):

                return "Scissors"
            elif (
                not index_straight
                and not middle_straight
                and not ring_straight
                and not pinky_straight
            ):
                return "Rock"
            return "Unknown"

        except Exception as e:
            print(f"Error during gesture recognition: {e}")
            return "Error"

    def process_frame(self, color_frame, depth_frame, get_3d_coords=True):

        if color_frame is None:
            print("HandDetector: Received None color frame.")
            return [], None

        image_rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(image_rgb)
        annotated_image = color_frame.copy()
        output_data = []

        if self.results.multi_hand_landmarks:
            intrinsics = None
            if get_3d_coords:
                intrinsics = self.camera.get_rs_intrinsics()
                if intrinsics is None:
                    print(
                        "Warning: Could not get camera intrinsics for 3D calculation."
                    )
                    get_3d_coords = False  # Fallback or skip

            for hand_index, hand_landmarks in enumerate(
                self.results.multi_hand_landmarks
            ):
                self.mp_draw.draw_landmarks(
                    annotated_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )

                if get_3d_coords and intrinsics:
                    # --- 3D Coordinate Calculation (Wrist) ---
                    pixel_coords = self._get_landmark_pixel_coords(
                        hand_landmarks, self.LM_WRIST, color_frame.shape
                    )
                    if pixel_coords:
                        px, py = pixel_coords
                        depth_value = self._get_depth_at_pixel(depth_frame, px, py)
                        point_3d = self._deproject_pixel_to_point_safe(
                            intrinsics, pixel_coords, depth_value
                        )
                        output_data.append(point_3d)  # Appends numpy array or None
                        if point_3d is not None:
                            # Create a 4x4 homogeneous transformation matrix (rotation = identity)
                            transform_matrix = np.eye(4)
                            transform_matrix[0:3, 3] = point_3d

                            # Transform hand position relative to origin tag if coordinate system is available
                            if self.coord_sys is not None and self.coord_sys.get_origin_transform() is not None:
                                # Get the origin transformation
                                origin_inv = self.coord_sys.get_origin_transform()

                                # Transform hand position to origin frame
                                transformed_matrix = origin_inv @ transform_matrix

                                # Apply coordinate system transformation (same as in CoordinateSystem.transform_poses)
                                o_to_t = transformed_matrix
                                offset_x = self.coord_sys.offset_x
                                offset_y = self.coord_sys.offset_y
                                offset_z = self.coord_sys.offset_z
                                bias_x = self.coord_sys.bias_x
                                bias_y = self.coord_sys.bias_y
                                bias_z = self.coord_sys.bias_z

                                # Apply the same transformation as for markers
                                final_matrix = np.array(
                                    [
                                        [
                                            o_to_t[1, 1],
                                            -o_to_t[1, 0],
                                            o_to_t[1, 2],
                                            o_to_t[1, 3] + bias_x - offset_x,
                                        ],
                                        [
                                            -o_to_t[0, 1],
                                            o_to_t[0, 0],
                                            -o_to_t[0, 2],
                                            -o_to_t[0, 3] + bias_y - offset_y,
                                        ],
                                        [
                                            o_to_t[2, 1],
                                            -o_to_t[2, 0],
                                            o_to_t[2, 2],
                                            o_to_t[2, 3] + bias_z - offset_z,
                                        ],
                                        [0, 0, 0, 1],
                                    ]
                                )

                                # Return the final matrix as the main result
                                output_data = final_matrix

                                # Get transformed position for display
                                transformed_point = final_matrix[0:3, 3]
                                display_text = f"Hand {hand_index}: ({transformed_point[0]:.2f},{transformed_point[1]:.2f},{transformed_point[2]:.2f})m (origin)"
                            else:
                                # If no coordinate system, just use camera frame matrix
                                output_data = transform_matrix
                                display_text = f"Hand {hand_index}: ({point_3d[0]:.2f},{point_3d[1]:.2f},{point_3d[2]:.2f})m (camera)"

                            # Display position on image
                            cv2.putText(
                                annotated_image,
                                display_text,
                                (px, py - 25),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA,
                            )
                        else:
                            output_data.append(None)

                    else:
                        output_data.append(None)

                else:
                    # --- Gesture Recognition ---
                    gesture = self._recognize_gesture(hand_landmarks, color_frame.shape)
                    # For gesture recognition, we return the gesture string instead of a matrix
                    output_data = gesture
                    # Draw gesture text near the wrist
                    wrist_coords = self._get_landmark_pixel_coords(
                        hand_landmarks, self.LM_WRIST, color_frame.shape
                    )
                    text_pos = (
                        (wrist_coords[0] - 40, wrist_coords[1] - 10)
                        if wrist_coords
                        else (50, 50 + hand_index * 30)
                    )  # Fallback position
                    cv2.putText(
                        annotated_image,
                        f"Hand {hand_index}: {gesture}",
                        text_pos,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

        return output_data, annotated_image

    def __del__(self):
        """Clean up resources."""
        if hasattr(self, "hands"):
            self.hands.close()
        print("HandDetector resources released.")
