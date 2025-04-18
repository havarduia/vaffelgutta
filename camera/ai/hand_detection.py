import cv2
import mediapipe as mp
import numpy as np
import os
import yaml

from robot.tools.file_manipulation import Jsonreader
from camera.coordinates import CoordinateSystem


class HandDetector:
    """Hand detection and tracking using MediaPipe."""

    def __init__(self, camera, coordinate_system=None):
        """Initialize with camera and optional coordinate system."""

        self.camera = camera

        self.coordinate_system = coordinate_system if coordinate_system is not None else CoordinateSystem()

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=1,
        )

        self.json_reader = Jsonreader()
        self.landmark_ids_for_position = [0, 1, 5, 9, 13, 17]  # Wrist, thumb, finger bases
        self.camera_matrix, self.dist_coeffs = self.camera.get_calibration()
        self.last_positions = []
        self.max_positions = 5
        self.min_positions = 3

        self.load_hand_bias_from_config()

    def depth_to_meters(self, u, v, z):
        """Convert pixel coordinates and depth to 3D point in meters."""
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        Z_in_meters = z / 1000
        X = (u - cx) * Z_in_meters / fx
        Y = (v - cy) * Z_in_meters / fy
        return np.array([X, Y, Z_in_meters])

    def get_depth_at_point(self, depth_frame, cx, cy, window_size=5):
        """Get robust depth by averaging a window around the point."""

        if window_size % 2 == 0:
            window_size += 1

        half_size = window_size // 2
        h, w = depth_frame.shape

        x_start = max(0, cx - half_size)
        x_end = min(w, cx + half_size + 1)
        y_start = max(0, cy - half_size)
        y_end = min(h, cy + half_size + 1)
        window = depth_frame[y_start:y_end, x_start:x_end]
        valid_depths = window[window > 0]

        if len(valid_depths) > 0:
            return np.median(valid_depths)
        else:
            return None

    def filter_position(self, new_position):
        """Apply temporal filtering to reduce jitter in hand position."""
        self.last_positions.append(new_position)
        if len(self.last_positions) > self.max_positions:
            self.last_positions.pop(0)
        if len(self.last_positions) >= self.min_positions:
            positions = np.array(self.last_positions)
            weights = np.linspace(0.5, 1.0, len(positions))
            weighted_positions = positions * weights[:, np.newaxis]
            filtered_position = np.sum(weighted_positions, axis=0) / np.sum(weights)

            return filtered_position
        else:
            return new_position

    def get_hand_position(self, landmarks, depth_frame, image_shape):
        """Calculate 3D hand position from landmarks and depth data."""
        h, w = image_shape
        points = []
        valid_landmarks = 0

        for idx in self.landmark_ids_for_position:
            lm = landmarks[idx]
            cx, cy = int(lm.x * w), int(lm.y * h)

            if 0 <= cx < depth_frame.shape[1] and 0 <= cy < depth_frame.shape[0]:
                z = self.get_depth_at_point(depth_frame, cx, cy)
                if z is not None:
                    point_in_meters = self.depth_to_meters(cx, cy, z)
                    points.append(point_in_meters)
                    valid_landmarks += 1
        if valid_landmarks < 3:
            return None
        hand_position = np.mean(points, axis=0)
        camera_frame_position = np.array(
            [
                hand_position[2],
                hand_position[0],
                hand_position[1]
            ]
        )

        return camera_frame_position

    def transform_position(self, position):
        """Transform position from camera frame to robot coordinate system."""

        matrix = np.eye(4)
        matrix[:3, 3] = position

        origin_inv = self.coordinate_system.get_origin_transform()
        if origin_inv is not None:
            try:
                o_to_t = origin_inv @ matrix

                transformed_matrix = np.array(
                    [
                        [
                            o_to_t[1, 1],
                            -o_to_t[1, 0],
                            o_to_t[1, 2],
                            o_to_t[1, 3]
                            + self.coordinate_system.bias_x
                            - self.coordinate_system.offset_x
                            + self.hand_bias_x,
                        ],
                        [
                            -o_to_t[0, 1],
                            o_to_t[0, 0],
                            -o_to_t[0, 2],
                            -o_to_t[0, 3]
                            + self.coordinate_system.bias_y
                            - self.coordinate_system.offset_y
                            + self.hand_bias_y,
                        ],
                        [
                            o_to_t[2, 1],
                            -o_to_t[2, 0],
                            o_to_t[2, 2],
                            o_to_t[2, 3]
                            + self.coordinate_system.bias_z
                            - self.coordinate_system.offset_z
                            + self.hand_bias_z,
                        ],
                        [0, 0, 0, 1],
                    ]
                )

                transformed_position = transformed_matrix[:3, 3]

                filtered_position = self.filter_position(transformed_position)

                return filtered_position, transformed_matrix
            except Exception:
                return None, None
        else:
            biased_position = position + np.array([
                self.hand_bias_x,
                self.hand_bias_y,
                self.hand_bias_z,
            ])

            biased_matrix = np.eye(4)
            biased_matrix[:3, 3] = biased_position

            return biased_position, biased_matrix

    def load_hand_bias_from_config(self):
        """Load hand bias values from the config file."""
        try:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), "config", "config.yaml"
            )
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            self.hand_bias_x = float(config.get("hand_bias_x", 0.0))
            self.hand_bias_y = float(config.get("hand_bias_y", 0.0))
            self.hand_bias_z = float(config.get("hand_bias_z", 0.0))
        except Exception as e:
            print(f"Error loading hand bias: {e}")
            self.hand_bias_x = 0.0
            self.hand_bias_y = 0.0
            self.hand_bias_z = 0.0

    def set_hand_bias(self, x=None, y=None, z=None):
        """Set bias values for hand position adjustment and save to config file."""

        if x is not None:
            self.hand_bias_x = float(x)
        if y is not None:
            self.hand_bias_y = float(y)
        if z is not None:
            self.hand_bias_z = float(z)

        try:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), "config", "config.yaml"
            )
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            config["hand_bias_x"] = self.hand_bias_x
            config["hand_bias_y"] = self.hand_bias_y
            config["hand_bias_z"] = self.hand_bias_z

            with open(config_path, "w") as f:
                yaml.dump(config, f, default_flow_style=False)

        except Exception as e:
            print(f"Error saving hand bias: {e}")

    def save_hand_matrix(self, matrix, filename="hand_position"):
        """Save hand transformation matrix to a JSON file."""

        if isinstance(matrix, np.ndarray):
            matrix = matrix.tolist()
        matrix_dict = {"matrix": matrix}
        self.json_reader.write(filename, matrix_dict)

    def process_frame(self, frame=None, depth_frame=None):
        """Process a frame to detect hands and return results."""

        if frame is None:
            frame = self.camera.get_color_frame()
        if depth_frame is None:
            depth_frame = self.camera.get_depth_frame()

        if frame is None or depth_frame is None:
            return None, None, None

        # Create a copy of the frame for processing
        process_frame = frame.copy()

        image_rgb = cv2.cvtColor(process_frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        return results, frame, depth_frame

    def draw_simple_landmarks(self, frame, results):
        """Draw minimal hand landmarks on the frame."""
        if results is None or frame is None:
            return frame

        vis_frame = frame.copy()
        # Just draw hand landmarks if detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw basic hand skeleton
                self.mp_drawing.draw_landmarks(
                    vis_frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=1)
                )

        return vis_frame

    def start(self, filename="hand_position", input_image=None, raw_tags=None):
        """Detect hand position and save it to a JSON file."""
        try:
            if raw_tags is not None and self.coordinate_system.origin_id in raw_tags:
                origin_matrix = raw_tags[self.coordinate_system.origin_id]
                origin_inv = np.linalg.inv(np.array(origin_matrix))
                self.coordinate_system.last_origin_inv = origin_inv
            results, frame, depth_frame = self.process_frame()

            if frame is None or depth_frame is None:
                return False, None

            if input_image is not None and input_image.shape[:2] == frame.shape[:2]:
                annotated_frame = self.draw_simple_landmarks(input_image.copy(), results)
            else:
                annotated_frame = self.draw_simple_landmarks(frame.copy(), results)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    try:
                        camera_pos = self.get_hand_position(
                            hand_landmarks.landmark, depth_frame, frame.shape[:2]
                        )

                        if camera_pos is not None:
                            pos, matrix = self.transform_position(camera_pos)

                            if pos is not None and matrix is not None:
                                self.save_hand_matrix(matrix, filename)
                                np.set_printoptions(precision=3, suppress=True)
                                print(np.array(matrix))

                                return True, annotated_frame
                    except Exception:
                        pass

            return False, annotated_frame
        except Exception:
            return False, input_image if input_image is not None else None
