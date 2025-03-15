from os import getcwd
import numpy as numphy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def convert_box(box):
    lower_corner = box[0]
    higher_corner = box[1]
    center =[lower_corner[ind] +1*(higher_corner[ind] - lower_corner[ind])/2 for ind in range(len(lower_corner))]
    length = higher_corner[0] - lower_corner[0]
    width = higher_corner[1] - lower_corner[1]
    height = higher_corner[2] - lower_corner[2]

    box = {
        "x":center[0],
        "y":center[1], # y left when arm forward
        "z":center[2],
        "depth": height,
        "width": length,
        "height": width
        #"depth":0.005,# z
        #"width":0.005,# x
        #"height":0.005# y
    }
    return box
    
def read_boxes():
    path = getcwd()
    path += "/robot/assets/boundingboxes/robot.py"
    with open(path,"r") as file:
        box_list: dict = eval(file.read(), {"np":numphy})
    boxes = []  
    for key in box_list.keys():
        boxes.append(convert_box(box_list[key]))
    return boxes


class BoxVisualizer(Node):
    def __init__(self, boxes):
        super().__init__('box_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'box_markers', 10)
        self.boxes = boxes  # initial set of boxes
        # Publish markers every second like a relentless hot drop
        self.timer = self.create_timer(1.0, self.publish_boxes)

    def publish_boxes(self):
        marker_array = MarkerArray()

        for i, box in enumerate(self.boxes):
            marker = Marker()
            marker.header.frame_id = "world"  # keep it in sync with your RViz fixed frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Setting up the pose and scale like you're customizing your loadout
            marker.pose.position.x = box['x']
            marker.pose.position.y = box['y']
            marker.pose.position.z = box['z']
            marker.scale.x = box['width']
            marker.scale.y = box['height']
            marker.scale.z = box['depth']

            # Color it up with that sexy green glow
            marker.color.a = 0.5
            marker.color.r = 0.4+0.01*i
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.lifetime.sec = 0  # keep it around indefinitely
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
        #self.get_logger().info("Published marker array with that steamy finesse.")

    def update_boxes(self, boxes):
        # Update the boxes that will be published in the next cycle.
        self.boxes = boxes


