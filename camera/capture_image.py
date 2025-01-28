import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Initialize the pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream color frames
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    # Capture 10 frames as an example (you can adjust this as needed)
    for i in range(30):
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get color frame
        color_frame = frames.get_color_frame()

        # Convert image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Generate a unique filename using a timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f'captured_image_{timestamp}_{i}.png'

        # Display the image in a window
        cv2.imshow('Color Image', color_image)

        # Save the image with a unique filename
        cv2.imwrite(filename, color_image)
        print(f'Image saved as {filename}')

        # Wait for a short time before capturing the next image
        key = cv2.waitKey(1000)  # 1 second delay between captures

        # Check if the user presses the 'q' key to quit the loop
        if key == ord('q'):
            print("Exiting capture loop.")
            break

finally:
    # Stop streaming
    pipeline.stop()

# Clean up OpenCV windows
cv2.destroyAllWindows()
