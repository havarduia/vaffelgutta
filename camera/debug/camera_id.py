import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

if not devices:
    print("No RealSense devices found.")
else:
    print("Connected devices:")
    for dev in devices:
        print(f"- {dev.get_info(rs.camera_info.serial_number)}")
