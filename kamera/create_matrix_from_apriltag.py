import cv2
import numpy as np
import pyrealsense2 as rs
import pupil_apriltags
import logging

def setup_camera(width=1280, height=720, fps=60):
    """Initialize and configure the RealSense pipeline."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    return pipeline, config

def align_frames(pipeline):
    """Align depth and color frames."""
    align = rs.align(rs.stream.color)
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    return aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()

def detect_apriltags(gray_image, camera_matrix, dist_coeffs, tag_size):
    """Detect AprilTags in the given grayscale image."""
    detector = pupil_apriltags.Detector(families="tag36h11")
    camera_params = [camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]]
    return detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

def save_transformation_matrix(tags, output_file="transformation_matrix.txt"):
    """Extract and save the transformation matrix of the first detected tag."""
    if not tags:
        logging.warning("No AprilTag detected.")
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
    logging.info(f"Transformation matrix saved to {output_file}:\n{transformation_matrix}")

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

    logging.basicConfig(level=logging.INFO)
    pipeline, config = setup_camera()

    try:
        pipeline.start(config)
        logging.info("Camera pipeline started.")

        color_frame, depth_frame = align_frames(pipeline)
        if not color_frame or not depth_frame:
            raise RuntimeError("Frames not available")

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = detect_apriltags(gray, camera_matrix, dist_coeffs, tag_size)
        save_transformation_matrix(tags, output_file)

    except Exception as e:
        logging.error(f"Error during processing: {e}")
    finally:
        pipeline.stop()
        logging.info("Camera pipeline stopped.")
