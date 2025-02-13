import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from math import radians

class PublishTF(Node):
    def __init__(self):
        super().__init__('publish_tf_node')
        
        # Creating a tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a timer to broadcast the transform at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Set up the transform matrix (Elon)
        self.transform = geometry_msgs.msg.TransformStamped()
        self.transform.header.frame_id = 'world'
        self.transform.child_frame_id = 'robot_frame'
        
        # Transformation matrix elements (Elon)
        self.transform.transform.translation.x = 0.08829702
        self.transform.transform.translation.y = -0.00673786
        self.transform.transform.translation.z = -0.08829702
        self.transform.transform.rotation.x = 0.02320824
        self.transform.transform.rotation.y = -0.13651191
        self.transform.transform.rotation.z = 0.13617325
        self.transform.transform.rotation.w = 0.99036654

    def timer_callback(self):
        # Update the timestamp and publish the transform
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.transform)
        self.get_logger().info('Publishing transform to RViz2.')

def main(args=None):
    rclpy.init(args=args)
    node = PublishTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
