import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('collision_object_publisher')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.collisionobjects = dict()
        self.ignored_objects = list()

    def _publish_collision_objects(self):
        for (name, endpoints) in self.collisionobjects.items():

            endpoint_min, endpoint_max = endpoints

            dim_x = endpoint_max[0] - endpoint_min[0]
            dim_y = endpoint_max[1] - endpoint_min[1]
            dim_z = endpoint_max[2] - endpoint_min[2]
            dimensions = [dim_x, dim_y, dim_z]

            position = endpoint_min.copy()
            position[0] += dim_x/2
            position[1] += dim_y/2
            position[2] += dim_z/2

            shape = SolidPrimitive()
            shape.type = SolidPrimitive.BOX
            shape.dimensions = dimensions
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.w = 1.0  # Neutral orientation
            obj = CollisionObject()
            obj.header = Header()
            obj.header.frame_id = "world"  
            obj.id = name
            obj.primitives.append(shape)
            obj.primitive_poses.append(pose)
            
            if name in self.ignored_objects:
                obj.operation = CollisionObject.REMOVE
            else:
                obj.operation = CollisionObject.ADD           
            
            self.publisher.publish(obj)        
        return 

    def publish_collisionobjects(self, collisionobjects: dict, ignore: list[str] = None):
        self.collisionobjects = collisionobjects
        self.ignored_objects = ignore
        self._publish_collision_objects()
        rclpy.spin_once(self)
        self.ignored_objects = None 

