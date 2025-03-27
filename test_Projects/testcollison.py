import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('collision_object_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, 'planning_scene', 10)
        # Publish once (or set up a timer for repeated publishing)
        self.publish_collision_object()

    def publish_collision_object(self):
        co = CollisionObject()
        co.header.frame_id = "base_link"  # Use your planning frame here
        co.id = "my_box"

        # Define a box shape
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.2, 0.2, 0.2]  # width, depth, height

        co.primitives.append(box)

        # Define the pose of the box
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0  # valid quaternion
        co.primitive_poses.append(pose)

        co.operation = CollisionObject.ADD

        self.publisher_.publish(co)
        self.get_logger().info("Published collision object.")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjectPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)  # allow time for the message to be published
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
