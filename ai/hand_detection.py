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
        self.landmark_ids_for_position = [0, 5, 9, 13, 17]

        self.camera_matrix, self.dist_coeffs = self.camera._get_calibration()
        self.camera_origin = camera_origin

    def set_camera_origin(self, origin):
        self.camera_origin = np.array(origin)

    def depth_to_meters(self, u, v, z):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        Z_in_meters = z / 1000  # Convert mm to meters
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

        hand_position = np.mean(points, axis=0)

        ros_position = np.array([
            hand_position[2],
            -hand_position[0],
            hand_position[1]
        ])

        adjusted_position = ros_position - self.camera_origin
        return adjusted_position

    def create_transformation_matrix(self, position):
        matrix = np.eye(4)
        matrix[:3, 3] = position
        return matrix

    def save_hand_matrix(self, matrix, filename="hand_position"):
        matrix_dict = {'matrix': matrix.tolist()}
        self.json_reader.write(filename, matrix_dict)

    def start(self, filename="hand_position"):
        frame = self.camera._get_image()
        depth_frame = self.camera._get_depth_image()

        if frame is None or depth_frame is None:
            print("No image!")
            return

        frame = cv2.flip(frame, 1)
        depth_frame = cv2.flip(depth_frame, 1)
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                pos = self.get_hand_position(
                    hand_landmarks.landmark,
                    depth_frame,
                    frame.shape[:2]
                )

                if pos is not None:
                    x, y, z = pos
                    print(f"Hand position (ROS origin): x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
                    matrix = self.create_transformation_matrix(pos)
                    self.save_hand_matrix(matrix, filename)
                    return  # Done after first valid save
        else:
            print("No hand detected.")
