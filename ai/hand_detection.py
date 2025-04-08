import cv2
import mediapipe as mp
import numpy as np
import json
from robot.tools.file_manipulation import Jsonreader

class HandDetector:
    def __init__(self, camera, json_writer: Jsonreader, json_filename: str):
        self.camera = camera
        self.json_writer = json_writer
        self.json_filename = json_filename

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

        self.landmark_ids_for_position = [0, 5, 9, 13, 17]
        self.camera_matrix, self.dist_coeffs = self.camera.get_calibration()

    def depth_to_meters(self, u, v, z):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        Z_in_meters = z / 1000
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
                    point_in_meters = self.depth_to_meters(cx, cy, z)
                    points.append(point_in_meters)

        if not points:
            return None

        return np.mean(points, axis=0)

    def save_transform_matrix(self, position: np.ndarray):
        """ Save 4x4 matrix with identity rotation and hand position to JSON file. """
        transform = np.eye(4)
        transform[0:3, 3] = position  # Set translation part

        transform_dict = {
            "hand": transform.tolist()
        }

        self.json_writer.write(self.json_filename, transform_dict)
        print(f"Transformation matrix written to {self.json_writer.directory_path}{self.json_filename}.json")

    def start(self):
        print("Starting hand detection...")
        while True:
            frame = self.camera.get_image()
            depth_frame = self.camera.get_depth_image()

            if frame is None or depth_frame is None:
                print("No frame received from camera.")
                continue

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
                        print(f"Hand position: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")

                        # Save matrix to file
                        self.save_transform_matrix(pos)

                        # Optional visualization
                        u = int(hand_landmarks.landmark[0].x * frame.shape[1])
                        v = int(hand_landmarks.landmark[0].y * frame.shape[0])
                        cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)
                        cv2.putText(frame, f"{z:.3f}m", (u, v - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("AI Hand Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
