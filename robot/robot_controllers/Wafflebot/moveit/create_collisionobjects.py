import rclpy
from rclpy.node import Node
import rclpy.service
from std_srvs.srv import Trigger


class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__("Collision_publisher_service_caller")
        self.client = self.create_client(
            srv_type=Trigger,
            srv_name="publish_boxes"
            ) 

    def publish_collisionobjects(self):
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return bool(future.result().success)
        
    