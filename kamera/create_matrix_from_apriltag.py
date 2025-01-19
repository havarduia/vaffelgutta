import cv2
import numpy as np
import pyrealsense2 as rs
import pupil_apriltags

def save_apriltag_matrix(output_file="transformation_matrix.txt", camera_matrix=None, dist_coeffs=None, tag_size=0.1):
    """
    Detect an AprilTag and save its 4x4 transformation matrix to a file.

    Args:
        output_file: File to save the transformation matrix.
        camera_matrix: Camera intrinsic matrix (3x3).
        dist_coeffs: Distortion coefficients (1x5).
        tag_size: Size of the AprilTag in meters.
    """
    # Ensure camera calibration data is provided
    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("Camera calibration data (camera_matrix, dist_coeffs) must be provided.")

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 60)
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)
    detector = pupil_apriltags.Detector(families="tag36h11")

    try:
        # Wait for frames and align them
        frames = align.process(pipeline.wait_for_frames())
        color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()

        if not color_frame or not depth_frame:
            raise RuntimeError("Frames not available")

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = detector.detect(gray, estimate_tag_pose=True, camera_params=[camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]], tag_size=tag_size)
        if not tags:
            print("No AprilTag detected.")
            return

        # Extract pose of the first detected tag
        tag = tags[0]
        pose_R = np.array(tag.pose_R).reshape(3, 3)  # Rotation matrix
        pose_t = np.array(tag.pose_t).flatten()      # Translation vector

        # Build transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = pose_R
        transformation_matrix[:3, 3] = pose_t

        # Save the transformation matrix to a file
        np.savetxt(output_file, transformation_matrix, fmt="%.6f")
        print(f"Transformation matrix saved to {output_file}:\n{transformation_matrix}")

    finally:
        pipeline.stop()
