import pyrealsense2 as rs
import cv2
import numpy as np


def main():
    # Create a pipeline
    pipeline = rs.pipeline()
    
    # Configure the pipeline to enable the RGB camera
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 60)
    
    # Start the pipeline
    pipeline.start(config)
    
    try:
        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            # Convert the color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Display the RGB image using OpenCV
            cv2.imshow('RealSense RGB Camera', color_image)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Stop the pipeline
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
