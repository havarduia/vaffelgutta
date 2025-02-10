import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from tf_transformations import quaternion_matrix
import numpy as np

# File path to save the matrix
FILE_PATH = "/home/vaffel/git/vaffelgutta/robot_workspace/assets/positions/camera_readings.py"

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscribed to /tag_detections')

    def listener_callback(self, msg):
        if not msg.detections:
            self.get_logger().info('No AprilTags detected')
            return

        for detection in msg.detections:
            pose = detection.pose.pose.pose  # Extract Pose

            quaternion = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            
            # Convert Quaternion to Rotation Matrix
            rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

            # Extract Translation
            translation = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])

            # Construct Full 4x4 Transformation Matrix
            transformation_matrix = np.eye(4)  # Identity 4x4
            transformation_matrix[:3, :3] = rotation_matrix  # Set rotation
            transformation_matrix[:3, 3] = translation       # Set translation

            # Round to 8 decimals
            transformation_matrix = np.round(transformation_matrix, 8)

            # Format as Python list
            matrix_str = "elon=([\n"
            for row in transformation_matrix:
                row_str = "  [" + ", ".join(f"{x:.8f}" for x in row) + "],"
                matrix_str += row_str + "\n"
            matrix_str += "])\n"

            # Print
            self.get_logger().info(f'\n{matrix_str}')

            # Save to file
            with open(FILE_PATH, "w") as f:
                f.write(matrix_str)

            self.get_logger().info(f'Saved matrix to {FILE_PATH}')

def run_camera(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
