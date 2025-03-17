from camera.camera_config_loader import ConfigLoader
from camera.print import print_blue, print_error
from robot.tools.file_manipulation import Jsonreader
import numpy as numphy  

class CoordinateSystem:
    def __init__(self, aruco_instance, config_loader):

        self.aruco = aruco_instance
        self.config_loader = config_loader
        self.marker_length = config_loader.get("marker_length")
        self.origin_id = config_loader.get("origin_id")
        self.transformations = self.aruco.estimate_pose()

    def transformation_origin_to_tag(self):
        """Transforms the origin marker to all detected tag transformations."""
        origin_id = self.config_loader.get("origin_id")
        bias_x, bias_y, bias_z = (self.config_loader.get(f"bias_{axis}") for axis in "xyz")
        offset_x, offset_y, offset_z = (self.config_loader.get(f"offset_{axis}") for axis in "xyz")

        if origin_id not in self.transformations:
            print_error(f"Error: Marker {origin_id} not detected! 👺")
            return {}

        origin_inv = numphy.linalg.inv(self.transformations[origin_id])
        tags = {}

        for tag_id, transformation in self.transformations.items():
            tag_id = int(tag_id)
            if tag_id == origin_id:
                continue

            origin_to_tag = origin_inv @ transformation
            
            # Reorder transformation matrix
            reordered_matrix = numphy.array([
                [ origin_to_tag[1, 1], -origin_to_tag[1, 0],  origin_to_tag[1, 2], origin_to_tag[1, 3] + bias_x - offset_x],
                [-origin_to_tag[0, 1],  origin_to_tag[0, 0], -origin_to_tag[0, 2], origin_to_tag[0, 3] + bias_y - offset_y],
                [ origin_to_tag[2, 1], -origin_to_tag[2, 0],  origin_to_tag[2, 2], origin_to_tag[2, 3] + bias_z - offset_z],
                [ 0,                   0,                    0,                   1]
            ])

            tags[tag_id] = reordered_matrix.tolist()

        return tags

    @staticmethod
    def init_tags(tags):
        """Converts input tags to a list of IDs."""
        return list(map(str, tags))

    def save_to_json(self, *allowed_tags):
        """Saves transformations to a JSON file while filtering allowed tags."""
        reader = Jsonreader()
        self.transformations = self.aruco.estimate_pose()
        tags = self.transformation_origin_to_tag()

        allowed_tags = self.init_tags(allowed_tags)
        reader.write("camera_readings", tags)
        data = reader.read("camera_readings")

        # Remove hallucinated tags
        for key in list(data.keys()):  
            if key not in allowed_tags:
                print(f"Removed hallucinated tag, id: {key}")    
                reader.pop("camera_readings", key)

        return list(data.keys())
