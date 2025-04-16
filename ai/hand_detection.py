import cv2
import mediapipe as mp
import numpy as np
from os import path as os_path
from robot.tools.file_manipulation import Jsonreader 

class HandDetector:
    def __init__(self, camera, camera_origin=np.array([0.28, -0.42, -0.4])):
        self.camera = camera

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.json_reader = Jsonreader()
        # Use stable points: wrist and finger bases (0, 5, 9, 13, 17)
        self.landmark_ids_for_position = [0, 5, 9, 13, 17]

        # Get camera calibration (camera matrix and distortion coefficients)
        self.camera_matrix, self.dist_coeffs = self.camera._get_calibration()

        # New camera origin (for example, (0, 0, 0) by default)
        self.camera_origin = camera_origin

    def set_camera_origin(self, origin):
        """Set a new camera origin."""
        self.camera_origin = np.array(origin)

    def depth_to_meters(self, u, v, z):
        """ Convert depth image coordinates to 3D world coordinates in meters. """
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # Convert depth (z) from pixels to real-world coordinates (meters)
        Z_in_meters = z / 1000  # Convert depth from mm to meters

        # Convert pixel coordinates (u, v) to world coordinates (X, Y, Z) in meters
        X = (u - cx) * Z_in_meters / fx
        Y = (v - cy) * Z_in_meters / fy
        return np.array([X, Y, Z_in_meters])

    def get_hand_position(self, landmarks, depth_frame, image_shape):
        h, w = image_shape
        points = []

        for idx in self.landmark_ids_for_position:
            lm = landmarks[idx]
            cx, cy = int(lm.x * w), int(lm.y * h)

            if 0 <= cx < depth_frame.shape[1] and 0 <= cy < depth_frame.shape[0]:
                z = depth_frame[cy, cx]
                if z > 0:
                    # Convert depth (z) to meters using the camera's intrinsic parameters
                    point_in_meters = self.depth_to_meters(cx, cy, z)
                    points.append(point_in_meters)

        if not points:
            return None

        # Average the valid 3D points to estimate hand position
        hand_position = np.mean(points, axis=0)

        # Apply ROS coordinate system transformation:
        # ROS frame: X -> forward (camera Z), Y -> left (camera X), Z -> up (camera Y)
        ros_position = np.array([
            hand_position[2],  # X in ROS frame = Z from the camera frame
            -hand_position[0], # Y in ROS frame = -X from the camera frame
            hand_position[1]   # Z in ROS frame = Y from the camera frame
        ])

        # Adjust the hand position based on the new camera origin (if necessary)
        adjusted_position = ros_position - self.camera_origin

        return adjusted_position

    def create_transformation_matrix(self, position):
        """Creates a 4x4 transformation matrix."""
        matrix = np.eye(4)
        matrix[:3, 3] = position  # Set the translation (position) in the last column
        return matrix
    
    def save_hand_matrix(self, matrix, filename="hand_position"):
        """Save the hand's transformation matrix to a JSON file."""
        # Convert matrix to a dictionary format for JSON serialization
        matrix_dict = {
            'matrix': matrix.tolist()  # Convert the NumPy array to a list of lists
        }
        self.json_reader.write(filename, matrix_dict)

    def start(self):
        while True:  # Keep the window open until a key is pressed
            frame = self.camera._get_image()
            depth_frame = self.camera._get_depth_image()

            if frame is not None or depth_frame is not None:

                frame = cv2.flip(frame, 1)
                depth_frame = cv2.flip(depth_frame, 1)
                image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image_rgb)

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            frame,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS
                        )

                        pos = self.get_hand_position(
                            hand_landmarks.landmark,
                            depth_frame,
                            frame.shape[:2]
                        )

                        if pos is not None:
                            x, y, z = pos
                            print(f"Hand position (ROS origin): x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")

                            # Create and save the transformation matrix
                            matrix = self.create_transformation_matrix(pos)
                            self.save_hand_matrix(matrix)

                            # Draw the position and matrix details
                            cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
                            cv2.putText(frame, f"{z:.3f}m", (int(x), int(y) - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow("AI Hand Detection", frame)

                # Wait for a key press to continue or close the window
                key = cv2.waitKey(1)  # Wait for 1 ms
                if key == ord('q'):  # Press 'q' to quit the loop
                    break
            else:  
                print("No image!")

        cv2.destroyAllWindows()  # Close the window after the loop ends
