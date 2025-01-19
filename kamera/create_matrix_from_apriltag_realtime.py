import cv2
import numpy as np
import pyrealsense2 as rs
import pupil_apriltags
import logging
import json

def setup_camera(width=1280, height=720, fps=60):
    """Initialize and configure the RealSense pipeline."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    return pipeline, config

def align_frames(pipeline, align):
    """Align depth and color frames."""
    for _ in range(5):  # Retry mechanism for frame alignment
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if color_frame and depth_frame:
            return color_frame, depth_frame
    raise RuntimeError("Failed to retrieve aligned frames.")

def detect_apriltags(gray_image, camera_matrix, dist_coeffs, tag_size):
    """Detect AprilTags in the given grayscale image."""
    detector = pupil_apriltags.Detector(families="tag36h11")
    camera_params = [camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]]
    return detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

def save_transformation_matrices(tags, output_file="transformation_matrices.json"):
    """Save transformation matrices of all detected tags to a JSON file."""
    if not tags:
        logging.warning("No AprilTags detected.")
        return

    transformations = []
    for i, tag in enumerate(tags):
        pose_R = np.array(tag.pose_R).reshape(3, 3).tolist()
        pose_t = np.array(tag.pose_t).flatten().tolist()

        transformation_data = {
            "tag_id": tag.tag_id,
            "rotation": pose_R,
            "translation": pose_t,
        }
        transformations.append(transformation_data)

    with open(output_file, "w") as f:
        json.dump(transformations, f, indent=4)
    logging.info(f"Saved transformation matrices for {len(tags)} tags to {output_file}.")

def main(camera_matrix, dist_coeffs, tag_size=0.1, output_file="transformation_matrices.json"):
    """
    Real-time AprilTag detection and transformation matrix saving.

    Args:
        camera_matrix: Camera intrinsic matrix (3x3).
        dist_coeffs: Distortion coefficients (1x5).
        tag_size: Size of the AprilTag in meters.
        output_file: File to save transformation matrices.
    """
    # Ensure camera calibration data is provided
    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("Camera calibration data (camera_matrix, dist_coeffs) must be provided.")

    logging.basicConfig(level=logging.INFO)
    pipeline, config = setup_camera()

    try:
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        logging.info("Camera pipeline started. Press 'q' to quit.")

        while True:
            try:
                # Align and retrieve frames
                color_frame, depth_frame = align_frames(pipeline, align)

                # Convert color frame to numpy array and grayscale
                color_image = np.asanyarray(color_frame.get_data())
                gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                # Detect AprilTags
                tags = detect_apriltags(gray_image, camera_matrix, dist_coeffs, tag_size)

                # Draw detections on the image
                for tag in tags:
                    for corner in tag.corners:
                        corner = tuple(map(int, corner))
                        cv2.circle(color_image, corner, 5, (0, 255, 0), -1)
                    center = tuple(map(int, tag.center))
                    cv2.circle(color_image, center, 5, (255, 0, 0), -1)
                    cv2.putText(color_image, f"ID: {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # Save transformation matrices
                save_transformation_matrices(tags, output_file)

                # Display the image
                cv2.imshow("AprilTag Detection", color_image)

                # Exit on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except RuntimeError as e:
                logging.warning(f"Frame processing error: {e}")
            except KeyboardInterrupt:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        logging.info("Camera pipeline stopped.")

