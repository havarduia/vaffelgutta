import os
import pyrealsense2 as rs
import numpy as np
import cv2

# Specify the folder where images will be saved
save_folder = "ai/images"

# Create the folder if it does not exist
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# Determine the starting count by checking existing files
existing_files = [f for f in os.listdir(save_folder) if f.endswith('.png')]
# Extract numbers from filenames that are purely numeric (e.g., "1.png", "2.png")
numbers = []
for filename in existing_files:
    base = os.path.splitext(filename)[0]
    if base.isdigit():
        numbers.append(int(base))
img_counter = max(numbers) + 1 if numbers else 1

# Configure the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a new set of frames from the camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Display the frame in a window
        cv2.imshow("RealSense", color_image)
        
        # Check for key presses
        key = cv2.waitKey(1) & 0xFF
        
        # If 's' is pressed, save the current frame with a sequential filename
        if key == ord('s'):
            filename = os.path.join(save_folder, f"{img_counter}.png")
            cv2.imwrite(filename, color_image)
            print(f"Image saved as {filename}")
            img_counter += 1
        # Press 'q' to exit the loop
        elif key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
