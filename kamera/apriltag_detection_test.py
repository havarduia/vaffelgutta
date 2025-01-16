import cv2
import pyrealsense2 as rs
import pupil_apriltags
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream color (RGB) and depth streams
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 60)  # 30 FPS, 1280x720 resolution
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)  # Depth stream
pipeline.start(config)

# Align depth to color frame
align = rs.align(rs.stream.color)

# Initialize the AprilTag detector
detector = pupil_apriltags.Detector()

# Main loop to capture frames from the L515 camera
try:
    while True:
        # Wait for a frame from the camera
        frames = pipeline.wait_for_frames()

        # Align depth frame to the color frame
        aligned_frames = align.process(frames)

        # Get the aligned color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue  # Skip iteration if frames are not available

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the depth frame to a numpy array (in meters)
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert the image to grayscale for AprilTag detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the grayscale image
        tags = detector.detect(gray)

        # Loop over the detected tags
        for tag in tags:
            # Get the corners of the detected tag
            corners = tag.corners.astype(int)

            # Get the center of the bounding box (average of corners)
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))

            # Check if the center is within the valid depth frame range
            if not (0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]):
                print(f"Center out of bounds: ({center_x}, {center_y})")
                continue

            # Get the depth value at the center of the tag
            distance = depth_frame.get_distance(center_x, center_y)

            # Skip invalid depth readings (e.g., distance is 0)
            if distance <= 0:
                print(f"Invalid depth reading at: ({center_x}, {center_y})")
                continue

            # Draw the bounding box (polygon) around the detected tag
            cv2.polylines(color_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

            # Put the tag ID and distance near the bounding box
            cv2.putText(color_image, f"ID: {tag.tag_id} Dist: {distance:.2f}m", 
                        (int(corners[0][0]), int(corners[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0, 255, 0), 2)

        # Display the image with detected tags and distances
        cv2.imshow("AprilTag Detection with RealSense", color_image)

        # Break the loop if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the RealSense pipeline and close any open windows
    pipeline.stop()
    cv2.destroyAllWindows()
