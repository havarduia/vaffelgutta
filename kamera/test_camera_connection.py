import pyrealsense2 as rs

def test_realsense_connection():
    # Create a context object to interact with all available RealSense devices
    context = rs.context()

    # Get the list of connected devices
    devices = context.query_devices()
    
    if len(devices) == 0:
        print("No RealSense devices detected.")
        return

    # Print out the details of the connected devices
    print(f"Number of connected devices: {len(devices)}")
    for i, device in enumerate(devices):
        print(f"Device {i+1}: {device.get_info(rs.camera_info.name)}")
        print(f"  Serial number: {device.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware version: {device.get_info(rs.camera_info.firmware_version)}")

    # Set up the pipeline to capture video from the camera
    pipeline = rs.pipeline()

    # Create a configuration object to configure the streams
    config = rs.config()
    
    # Enable color and depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 30 FPS
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    print("Streaming started...")

    try:
        # Wait for a frame from the camera
        frames = pipeline.wait_for_frames()

        # Get the color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Check if frames are valid
        if not color_frame or not depth_frame:
            print("Failed to capture frames.")
            return

        print(f"Captured color frame of size: {color_frame.get_width()} x {color_frame.get_height()}")
        print(f"Captured depth frame of size: {depth_frame.get_width()} x {depth_frame.get_height()}")

    finally:
        # Stop streaming when done
        pipeline.stop()

if __name__ == "__main__":
    test_realsense_connection()
