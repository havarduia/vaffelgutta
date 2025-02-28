import pyrealsense2 as rs
import numpy as np
import cv2

import pyrealsense2 as rs
import numpy as np

class Camera:
    def __init__(self, camera_id, x, y):
        self.isStreaming = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(camera_id)
        self.config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, 30)
        self.color_frame = None
        self.intrinsics = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.start_streaming()

    def __del__(self):
        self.stop_streaming()

    def start_streaming(self):
        if not self.isStreaming:
            profile = self.pipeline.start(self.config)
            self.isStreaming = True

            # Get intrinsics from the color stream
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            
            # Store intrinsics
            self.intrinsics = intrinsics
            self.camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            print(self.camera_matrix)
            self.dist_coeffs = np.array(intrinsics.coeffs[:5])

    def stop_streaming(self):
        if self.isStreaming:
            self.pipeline.stop()
            self.isStreaming = False

    def get_frame(self):
        if not self.isStreaming:
            return None
        frame = self.pipeline.wait_for_frames()
        return frame

    def np_frames(self, frame):
        self.color_frame = frame.get_color_frame()
        color_image = np.asanyarray(self.color_frame.get_data())
        return color_image










# Just for debugging
def main():
    cam1_id = "031422250347"
    cam2_id = "912112072861"

    camera1 = Camera(cam1_id, 1280, 720)
    camera2 = Camera(cam2_id, 1280, 720)

    while True:
        frame1 = camera1.get_frame()
        frame2 = camera2.get_frame()
        image1 = camera1.np_frames(frame1)
        image2 = camera2.np_frames(frame2)
        # Stack images horizontally
        stacked = cv2.hconcat([image1, image2])
        # Display the stacked image in one window
        cv2.imshow('Stacked Cameras', stacked)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
