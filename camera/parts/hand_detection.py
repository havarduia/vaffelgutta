import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp

class HandDetector:
    """
    Detects hands in frames via MediaPipe and computes 3D wrist position and transformation matrix.
    """

    def __init__(self, camera):

        self.camera = camera
        self.rs_intrinsics = camera.get_rs_intrinsics()
        self.depth_scale = camera.get_depth_scale()
        if not self.rs_intrinsics:
            raise ValueError("Camera intrinsics unavailable")
        if not self.depth_scale:
            raise ValueError("Invalid depth scale")

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.drawing_utils = mp.solutions.drawing_utils

    def process_frame(
        self, image=None
    ) -> tuple[np.ndarray, mp.solutions.hands.Hands, list[np.ndarray]]:
        """
        Captures frames, detects hands, draws landmarks, and returns image, results, and list of 4x4 transform matrices.

        Args:
            image: Optional pre-processed image to use. If provided, will use this image for hand detection
                  but still get a depth frame for 3D position calculation.
        """
        # Get frames from camera if no image is provided
        if image is None:
            color, depth = self.camera.get_aligned_frames()
            if color is None or depth is None:
                return None, None, []
        else:
            # If image is provided, we still need depth data for 3D position
            color = image
            _, depth = self.camera.get_aligned_frames()
            if depth is None:
                return color, None, []

        h, w, _ = color.shape
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        matrices = []
        if results.multi_hand_landmarks:
            for landmarks in results.multi_hand_landmarks:
                self.drawing_utils.draw_landmarks(
                    color, landmarks, mp.solutions.hands.HAND_CONNECTIONS
                )
                wrist = landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
                x_px, y_px = int(wrist.x * w), int(wrist.y * h)
                if 0 <= x_px < w and 0 <= y_px < h:
                    depth_mm = depth[y_px, x_px]
                    if depth_mm > 0:
                        z_m = depth_mm * self.depth_scale
                        x_cam, y_cam, z_cam = rs.rs2_deproject_pixel_to_point(
                            self.rs_intrinsics, [x_px, y_px], z_m
                        )
                        tx, ty, tz = z_cam, -x_cam, -y_cam
                        mat = np.eye(4)
                        mat[:3, 3] = [tx, ty, tz]
                        matrices.append(mat)
                        cv2.putText(
                            color,
                            f"X:{tx:.2f} Y:{ty:.2f} Z:{tz:.2f}",
                            (x_px + 10, y_px + 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (255, 255, 0),
                            1,
                        )
        return color, results, matrices

    def release(self):
        """Releases MediaPipe resources."""
        self.hands.close()
