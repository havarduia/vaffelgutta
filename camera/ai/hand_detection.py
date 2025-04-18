import cv2
import mediapipe as mp
import numpy as np
import os
import yaml

from robot.tools.file_manipulation import Jsonreader
from camera.coordinates import CoordinateSystem


class HandDetector:
    """Class for detecting and tracking hands using MediaPipe."""

    def __init__(self, camera, coordinate_system=None):
        """Initialize the hand detector.

        Args:
            camera: Camera instance that provides frames and calibration
            coordinate_system: Optional CoordinateSystem instance. If None, a new instance will be created.
        """
        self.camera = camera
        # Use the provided coordinate system or create a new one
        self.coordinate_system = coordinate_system if coordinate_system is not None else CoordinateSystem()

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=1,  # Use more complex model for better accuracy
        )

        self.json_reader = Jsonreader()
        # Key hand landmarks for position estimation
        self.landmark_ids_for_position = [0, 1, 5, 9, 13, 17]  # Wrist, thumb, and finger bases

        # Get camera calibration
        self.camera_matrix, self.dist_coeffs = self.camera.get_calibration()

        # Position filtering to reduce jitter
        self.last_positions = []
        self.max_positions = 5  # Number of positions to keep for filtering
        self.min_positions = 3  # Minimum positions needed for filtering

        # Load bias parameters from config file
        self.load_hand_bias_from_config()

    def depth_to_meters(self, u, v, z):
        """Convert pixel coordinates and depth to 3D point in meters."""
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        Z_in_meters = z / 1000  # Convert mm to meters
        X = (u - cx) * Z_in_meters / fx
        Y = (v - cy) * Z_in_meters / fy
        return np.array([X, Y, Z_in_meters])

    def get_depth_at_point(self, depth_frame, cx, cy, window_size=5):
        """Get a more robust depth measurement by averaging a small window around the point.

        Args:
            depth_frame: Depth frame from the camera
            cx, cy: Pixel coordinates
            window_size: Size of the window to average (must be odd)

        Returns:
            float: Average depth value, or None if no valid depth was found
        """
        if window_size % 2 == 0:
            window_size += 1  # Ensure window_size is odd

        half_size = window_size // 2
        h, w = depth_frame.shape

        # Define the window boundaries, ensuring they're within the frame
        x_start = max(0, cx - half_size)
        x_end = min(w, cx + half_size + 1)
        y_start = max(0, cy - half_size)
        y_end = min(h, cy + half_size + 1)

        # Extract the window
        window = depth_frame[y_start:y_end, x_start:x_end]

        # Filter out zero values (invalid depth measurements)
        valid_depths = window[window > 0]

        if len(valid_depths) > 0:
            # Use median to be more robust against outliers
            return np.median(valid_depths)
        else:
            return None

    def filter_position(self, new_position):
        """Apply temporal filtering to reduce jitter in hand position.

        Args:
            new_position: New hand position to filter

        Returns:
            numpy.ndarray: Filtered position
        """
        # Add the new position to our history
        self.last_positions.append(new_position)

        # Keep only the last max_positions
        if len(self.last_positions) > self.max_positions:
            self.last_positions.pop(0)

        # If we have enough positions, apply filtering
        if len(self.last_positions) >= self.min_positions:
            # Convert to numpy array for easier calculations
            positions = np.array(self.last_positions)

            # Apply weighted average (more weight to recent positions)
            weights = np.linspace(0.5, 1.0, len(positions))
            weighted_positions = positions * weights[:, np.newaxis]
            filtered_position = np.sum(weighted_positions, axis=0) / np.sum(weights)

            return filtered_position
        else:
            # Not enough positions for filtering, return the new position
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
                # Get a more robust depth measurement
                z = self.get_depth_at_point(depth_frame, cx, cy)
                if z is not None:
                    point_in_meters = self.depth_to_meters(cx, cy, z)
                    points.append(point_in_meters)
                    valid_landmarks += 1

        # Require at least 3 valid landmarks for a reliable position
        if valid_landmarks < 3:
            return None

        # Calculate the average position from valid points
        hand_position = np.mean(points, axis=0)

        # Convert to camera coordinate system
        camera_frame_position = np.array(
            [
                hand_position[2],  # Z in camera becomes X in camera frame
                hand_position[0],  # X in camera becomes Y in camera frame
                hand_position[1],  # Y in camera becomes Z in camera frame
            ]
        )

        return camera_frame_position

    def transform_position(self, position):
        """Transform position from camera frame to robot coordinate system."""
        # Create a 4x4 transformation matrix for the position
        matrix = np.eye(4)
        matrix[:3, 3] = position

        # Get the origin transformation if available
        origin_inv = self.coordinate_system.get_origin_transform()
        if origin_inv is not None:
            try:
                # Transform from camera frame to origin frame
                o_to_t = origin_inv @ matrix

                # Apply coordinate system transformation and offsets
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

                # Extract the position from the transformed matrix
                transformed_position = transformed_matrix[:3, 3]

                # Apply temporal filtering to reduce jitter
                filtered_position = self.filter_position(transformed_position)

                return filtered_position, transformed_matrix
            except Exception:
                # Error in transformation
                return None, None
        else:
            # If no origin transform is available, apply bias directly
            biased_position = position + np.array([
                self.hand_bias_x,
                self.hand_bias_y,
                self.hand_bias_z,
            ])

            # Create a matrix with the biased position
            biased_matrix = np.eye(4)
            biased_matrix[:3, 3] = biased_position

            return biased_position, biased_matrix

    def load_hand_bias_from_config(self):
        """Load hand bias values from the config file."""
        try:
            # Load config file
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), "config", "config.yaml"
            )
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Get hand bias values with defaults if not present
            self.hand_bias_x = float(config.get("hand_bias_x", 0.0))
            self.hand_bias_y = float(config.get("hand_bias_y", 0.0))
            self.hand_bias_z = float(config.get("hand_bias_z", 0.0))

            # Bias values loaded from config
        except Exception as e:
            # Log error but continue with defaults
            print(f"Error loading hand bias: {e}")
            # Set default values if loading fails
            self.hand_bias_x = 0.0
            self.hand_bias_y = 0.0
            self.hand_bias_z = 0.0

    def set_hand_bias(self, x=None, y=None, z=None):
        """Set bias values for hand position adjustment and save to config file.

        Args:
            x: Bias in x direction (forward/backward)
            y: Bias in y direction (left/right)
            z: Bias in z direction (up/down)
        """
        # Update bias values if provided
        if x is not None:
            self.hand_bias_x = float(x)
        if y is not None:
            self.hand_bias_y = float(y)
        if z is not None:
            self.hand_bias_z = float(z)

        # Save to config file
        try:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), "config", "config.yaml"
            )
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Update config with new values
            config["hand_bias_x"] = self.hand_bias_x
            config["hand_bias_y"] = self.hand_bias_y
            config["hand_bias_z"] = self.hand_bias_z

            # Write back to file
            with open(config_path, "w") as f:
                yaml.dump(config, f, default_flow_style=False)

            # Bias values saved to config
        except Exception as e:
            print(f"Error saving hand bias: {e}")

    def save_hand_matrix(self, matrix, filename="hand_position"):
        """Save hand transformation matrix to a JSON file."""
        # Ensure the matrix is in the correct format (list of lists)
        if isinstance(matrix, np.ndarray):
            matrix = matrix.tolist()
        matrix_dict = {"matrix": matrix}
        self.json_reader.write(filename, matrix_dict)

    def process_frame(self, frame=None, depth_frame=None):
        """Process a frame to detect hands and return results.

        Args:
            frame: Optional color frame. If None, gets a new frame from the camera.
            depth_frame: Optional depth frame. If None, gets a new frame from the camera.

        Returns:
            tuple: (results, frame, depth_frame) where results is the MediaPipe hand detection results
        """
        # Get frames from camera if not provided
        if frame is None:
            frame = self.camera.get_color_frame()
        if depth_frame is None:
            depth_frame = self.camera.get_depth_frame()

        if frame is None or depth_frame is None:
            # No image available
            return None, None, None

        # Create a copy of the frame for processing
        # We don't flip the frame anymore to keep consistent with marker detection
        process_frame = frame.copy()

        # Convert to RGB for MediaPipe
        image_rgb = cv2.cvtColor(process_frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        return results, frame, depth_frame

    def draw_simple_landmarks(self, frame, results):
        """Draw basic hand landmarks on the frame."""
        if results is None or frame is None:
            return frame

        # Create a copy of the frame to draw on
        vis_frame = frame.copy()

        # Draw origin status
        origin_id = self.coordinate_system.origin_id
        origin_transform = self.coordinate_system.get_origin_transform()
        status_color = (0, 255, 0) if origin_transform is not None else (0, 0, 255)
        cv2.putText(
            vis_frame,
            f"Origin (ID: {origin_id}): {'Detected' if origin_transform is not None else 'Not Detected'}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2
        )

        # Draw bias info
        cv2.putText(
            vis_frame,
            f"Bias: X={self.hand_bias_x:.3f}, Y={self.hand_bias_y:.3f}, Z={self.hand_bias_z:.3f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 165, 0),
            2
        )

        # Draw hand landmarks if detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    vis_frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                )

                # Highlight key landmarks
                h, w, _ = vis_frame.shape
                for idx in self.landmark_ids_for_position:
                    lm = hand_landmarks.landmark[idx]
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(vis_frame, (cx, cy), 8, (255, 0, 255), -1)  # Magenta filled circle

        return vis_frame

    def start(self, filename="hand_position", input_image=None, raw_tags=None):
        """Detect hand position and save it to a JSON file."""
        try:
            # Update coordinate system if raw_tags contains the origin marker
            if raw_tags is not None and self.coordinate_system.origin_id in raw_tags:
                origin_matrix = raw_tags[self.coordinate_system.origin_id]
                origin_inv = np.linalg.inv(np.array(origin_matrix))
                self.coordinate_system.last_origin_inv = origin_inv

            # Process frame
            results, frame, depth_frame = self.process_frame()

            if frame is None or depth_frame is None:
                return False, None

            # Draw landmarks on the appropriate frame
            if input_image is not None and input_image.shape[:2] == frame.shape[:2]:
                annotated_frame = self.draw_simple_landmarks(input_image.copy(), results)
            else:
                annotated_frame = self.draw_simple_landmarks(frame.copy(), results)

            # Process hand detection results
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    try:
                        # Get hand position in camera frame
                        camera_pos = self.get_hand_position(
                            hand_landmarks.landmark, depth_frame, frame.shape[:2]
                        )

                        if camera_pos is not None:
                            # Transform to robot coordinate system
                            pos, matrix = self.transform_position(camera_pos)

                            if pos is not None and matrix is not None:
                                # Save the transformation matrix
                                self.save_hand_matrix(matrix, filename)

                                # Print the matrix in a clean format
                                np.set_printoptions(precision=3, suppress=True)
                                print(np.array(matrix))

                                return True, annotated_frame
                    except Exception:
                        pass

            return False, annotated_frame
        except Exception:
            return False, input_image if input_image is not None else None
