import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
from camera.realsense import Camera
from camera.Config.misc import ConfigLoader

from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision import PoseLandmarker
from mediapipe.tasks.python.vision.pose_landmarker import PoseLandmarkerOptions
from mediapipe.tasks.python.vision import RunningMode
from mediapipe.calculators import image as mp_Image




class MultiPersonPoseWithDepth:
    def __init__(self, config_loader, model_path):
        self.camera = Camera(config_loader)
        
        # Set up multi-person pose estimation options.
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_poses=2,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_segmentation_masks=False
        )
        self.landmarker = PoseLandmarker.create_from_options(options)
        
        self.mp_drawing = mp.solutions.drawing_utils

        # RealSense: Align depth to color.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def start(self):
        self.camera.start_streaming()
        while True:
            frames = self.camera.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                print("Failed to capture frames.")
                break

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Convert to MediaPipe Image.
            mp_image = mp_Image(image_format=mp_Image.ImageFormat.SRGB, data=rgb_image)

            # Run the multi-person pose estimation.
            result = self.landmarker.detect(mp_image)
            if result.pose_landmarks:
                for person_landmarks in result.pose_landmarks:
                    self.mp_drawing.draw_landmarks(
                        color_image,
                        person_landmarks,
                        mp.solutions.pose.POSE_CONNECTIONS
                    )
                    for landmark in person_landmarks.landmark:
                        x = int(landmark.x * color_image.shape[1])
                        y = int(landmark.y * color_image.shape[0])
                        x = np.clip(x, 0, depth_image.shape[1] - 1)
                        y = np.clip(y, 0, depth_image.shape[0] - 1)
                        depth_value = depth_image[y, x]
                        depth_in_meters = depth_value / 1000.0
                        cv2.putText(color_image, f"{depth_in_meters:.2f}m", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Multi-Person Pose with Depth", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.camera.stop_streaming()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    config_loader = ConfigLoader('/home/havard/git/vaffelgutta/camera/Config/config.yaml')
    model_path = '/home/havard/git/vaffelgutta/timmy/pose_landmarker_lite.task'
    detector = MultiPersonPoseWithDepth(config_loader, model_path)
    detector.start()
