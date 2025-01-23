"""
The following is greg code used for debugging.
The conversation history is pasted in the bottom of the script.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_matrix
import numpy as np

class TFPublisher:
    _instance = None  # Singleton instance
    _node = None  # Store the node separately

    def __new__(cls, parent_frame="world", child_frame="robot"):
        """
        Ensure only one instance of TFPublisher exists.
        """
        if cls._instance is None:
            cls._instance = super(TFPublisher, cls).__new__(cls)
        return cls._instance

    def __init__(self, parent_frame="world", child_frame="robot"):
        """
        Initializes the TF publisher node only when necessary.
        """
        if TFPublisher._node is None:  # Create the node only if it doesn't exist
            # Create the node once here if rclpy.init() has been called externally
            if rclpy.ok():
                TFPublisher._node = rclpy.create_node('tf_publisher')  # Create node after ROS initialization
            else:
                print("[WARN] rclpy.init() was not called in this script but is assumed to be running.")
                TFPublisher._node = Node('tf_publisher')  # Fallback if ROS is assumed to be running

        self.broadcaster = tf2_ros.TransformBroadcaster(TFPublisher._node)
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.last_transform = None
    def broadcast_transform(self, matrix):
        """
        Sends a 4x4 transformation matrix as a TF broadcast, replacing the previous one.

        :param matrix: 4x4 numpy array representing the transformation.
        """
        t = TransformStamped()
        t.header.stamp = TFPublisher._node.get_clock().now().to_msg()  # Access the clock using the node
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        # Extract translation
        """NB! Changed indexing from [x,]to [][]"""
        t.transform.translation.x = matrix[0][3]
        t.transform.translation.y = matrix[1][3]
        t.transform.translation.z = matrix[2][3]
        # Extract rotation as quaternion
        quaternion = quaternion_from_matrix(matrix)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        # Overwrite the last transform and publish the new one
        self.last_transform = t
        self.broadcaster.sendTransform(t)

# Singleton instance
_tf_publisher = None

def publish_tf(matrix, parent_frame="world", child_frame="robot"):
    """
    External function to publish a TF, replacing the previous one.
    Ensures ROS is only initialized once.

    :param matrix: 4x4 numpy array for transformation.
    :param parent_frame: Parent frame ID.
    :param child_frame: Child frame ID.
    """
    global _tf_publisher
    if _tf_publisher is None:
        _tf_publisher = TFPublisher(parent_frame, child_frame)

    _tf_publisher.broadcast_transform(matrix)



#