import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import tf_transformations

# File path to save the matrix
FILE_PATH = "/home/havard/git/vaffelgutta/robot_workspace/assets/positions/camera_readings.py"

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10
        )
        self.get_logger().info('Apriltag Online')

    def listener_callback(self, msg):
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

            # Swap Z and X translation
            t = tf_transformations.Transform()
            t.transform.translation.x = float(transformation_matrix[2][3])
            t.transform.translation.y = float(transformation_matrix[1][3])
            t.transform.translation.z = -float(transformation_matrix[0][3])

            q = quaternion_from_matrix(transformation_matrix)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Format as Python list
            matrix_str = "elon=([\n"
            for row in transformation_matrix:
                row_str = "  [" + ", ".join(f"{x:.8f}" for x in row) + "],"
                matrix_str += row_str + "\n"
            matrix_str += "])\n"

            # Save to file
            with open(FILE_PATH, "w") as f:
                f.write(matrix_str)


def run_camera():
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
