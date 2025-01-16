import cv2
import numpy as np
import pyrealsense2 as rs
import pupil_apriltags

def save_apriltag_matrix(output_file="transformation_matrix.txt"):
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    
    # Configure the pipeline
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 60)
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    pipeline.start(config)
    
    # Align depth to color frame
    align = rs.align(rs.stream.color)
    
    # Initialize the AprilTag detector
    detector = pupil_apriltags.Detector()
    
    try:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        # Get frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            raise ValueError("No frames available")
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Convert image to grayscale for AprilTag detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        
        if not tags:
            print("No AprilTag detected.")
            return
        
        # Process first detected tag
        tag = tags[0]
        
        # Debugging: Check the detected tag and its pose_R
        print("Detected tag:", tag)
        print("pose_R:", tag.pose_R)

        # Check for valid pose_R (rotation matrix)
        if tag.pose_R is not None and len(tag.pose_R) == 9:
            rvec = np.array(tag.pose_R).reshape(3, 3)  # Rotation matrix
        else:
            print("Warning: Invalid or missing rotation matrix (pose_R), using solvePnP to estimate pose.")
            
            # Object points (real-world coordinates of the corners of the tag)
            tag_size = 0.1  # Adjust this to the actual size of your AprilTag in meters
            object_points = np.array([
                [0, 0, 0],  # bottom-left corner
                [tag_size, 0, 0],  # bottom-right corner
                [0, tag_size, 0],  # top-left corner
                [tag_size, tag_size, 0]  # top-right corner
            ], dtype=np.float32)

            # Image points (2D corners of the tag in the image)
            image_points = np.array(tag.corners, dtype=np.float32)

            # Camera intrinsics (get from your camera calibration)
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ], dtype=np.float32)

            # Distortion coefficients (if available, else assume no distortion)
            dist_coeffs = np.zeros(5)  # Assuming no lens distortion

            # SolvePnP to estimate pose (rotation and translation)
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            
            if success:
                print("Pose estimated using solvePnP.")
                # Convert rvec to rotation matrix
                rvec, _ = cv2.Rodrigues(rvec)
            else:
                print("Error estimating pose using solvePnP, using identity matrix.")
                rvec = np.eye(3)  # Default identity matrix if pose estimation fails
        
        # Get the center of the tag
        corners = tag.corners.astype(int)
        center_x = int(np.mean(corners[:, 0]))
        center_y = int(np.mean(corners[:, 1]))
        
        # Get depth at center
        distance = depth_frame.get_distance(center_x, center_y)
        if distance <= 0:
            raise ValueError("Invalid depth reading")
        
        # Compute 3D position
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        point = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], distance)
        tvec = np.array(point)  # Translation vector
        
        # Construct 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rvec
        transformation_matrix[:3, 3] = tvec
        
        # Save the matrix to a file
        with open(output_file, "w") as f:
            for row in transformation_matrix:
                f.write(" ".join(map(str, row)) + "\n")
        
        print(f"Transformation Matrix saved to {output_file}:")
        print(transformation_matrix)
    
    finally:
        pipeline.stop()
