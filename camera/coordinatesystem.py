from robot.tools.file_manipulation import Jsonreader
from camera.Config.misc import print_blue, print_error, ConfigLoader
import numpy as np

np.set_printoptions(precision=4)


class CoordinateSystem:
    def __init__(self, aruco, config_loader):
        if aruco is None:
            raise Exception("Aruco instance not found. Ensure Aruco is initialized after Camera.")

        self.aruco = aruco
        self.config_loader = config_loader
        config = {key: config_loader.get(key) for key in 
                  ["marker_length", "origin_id", "offset_x", "offset_y", "offset_z", "bias_x", "bias_y", "bias_z"]}

        self.marker_length = config["marker_length"]
        self.origin_id = config["origin_id"]
        self.offset_x, self.offset_y, self.offset_z = config["offset_x"], config["offset_y"], config["offset_z"]
        self.bias_x, self.bias_y, self.bias_z = config["bias_x"], config["bias_y"], config["bias_z"]
        self.last_origin_inv = None  # Initialize last known transformation

    def transformation_origin_to_tag(self):
        new_transformations = self.aruco.estimate_pose()

        # Use last known origin transformation if origin marker is not detected
        origin_inv = self.last_origin_inv
        if self.origin_id in new_transformations:
            origin_inv = np.linalg.inv(new_transformations[self.origin_id])
            self.last_origin_inv = origin_inv
        else:
            print_error("Could not find origin 👺")

        if origin_inv is None:
            return {}  # Return empty dict if no valid transformation is found

        tags = {}
        for tag_id, transformation in new_transformations.items():
            tag_id = int(tag_id)
            if tag_id == self.origin_id:
                continue

            o_to_t = origin_inv @ transformation

            transformed_matrix = np.array([
                [o_to_t[1][1], -o_to_t[1][0], o_to_t[1][2], o_to_t[1][3] + self.bias_x - self.offset_x],
                [-o_to_t[0][1], o_to_t[0][0], -o_to_t[0][2], -o_to_t[0][3] + self.bias_y - self.offset_y],
                [o_to_t[2][1], -o_to_t[2][0], o_to_t[2][2], o_to_t[2][3] + self.bias_z - self.offset_z],
                [0, 0, 0, 1],
            ]).tolist()

            tags[tag_id] = transformed_matrix

        return tags

    def init_tags(self, tags: str | int):
        all_tags = []
        for tag in tags:
            all_tags.append(str(tag))
        return all_tags

    def start(self, *allowed_tags: str | int) -> list[str]:
        config_loader = ConfigLoader()
        reader = Jsonreader()
        tags = self.transformation_origin_to_tag()

        allowed_tags = self.init_tags(allowed_tags)
        for id in tags.keys():
            id = str(id)

        reader.write("camera_readings", tags)
        data = reader.read("camera_readings")
        if "all" not in allowed_tags:
            for key in data.keys():
                #print(key)
                if not key in allowed_tags:
                    print(f"removed hallucinated tag, id: {key}")
                    reader.pop("camera_readings", key)