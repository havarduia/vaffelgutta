from robot.tools.file_manipulation import Jsonreader
from camera.Config.misc import print_blue, print_error, ConfigLoader
import numpy as numphy
numphy.set_printoptions(precision=4)


class CoordinateSystem:
    def __init__(self, aruco, config_loader):

        self.aruco = aruco
        if self.aruco is None:
            raise Exception(
                "Aruco instance not found. Ensure Aruco is initialized after Camera."
            )
        self.config_loader = config_loader
        self.marker_length = config_loader.get("marker_length")
        self.origin_id = config_loader.get("origin_id")
        self.origin_id = config_loader.get("origin_id")
        self.offset_x = config_loader.get("offset_x")
        self.offset_y = config_loader.get("offset_y")
        self.offset_z = config_loader.get("offset_z")
        self.bias_x = config_loader.get("bias_x")
        self.bias_y = config_loader.get("bias_y")
        self.bias_z = config_loader.get("bias_z")

    def transformation_origin_to_tag(self):
        self.new_transformations = self.aruco.estimate_pose()
        # If the origin marker is not detected, keep using the last known origin transformation
        if self.origin_id not in self.new_transformations:
            print_error("Could not find origin 👺")
            origin_inv = self.last_origin_inv  # Use cached inverse matrix
        else:
            origin = self.new_transformations[self.origin_id]
            origin_inv = numphy.linalg.inv(origin)
            self.last_origin_inv = origin_inv

        tags = {}

        for tag_id, transformation in self.new_transformations.items():
            tag_id = int(tag_id)
            if tag_id == self.origin_id:
                continue

            origin_to_tag = numphy.dot(origin_inv, transformation)

            xx, xy, xz = origin_to_tag[1][1], -origin_to_tag[0][1], origin_to_tag[2][1]
            yx, yy, yz = -origin_to_tag[1][0], origin_to_tag[0][0], -origin_to_tag[2][0]
            zx, zy, zz = origin_to_tag[1][2], -origin_to_tag[0][2], origin_to_tag[2][2]
            tx, ty, tz = origin_to_tag[1][3], -origin_to_tag[0][3], origin_to_tag[2][3]

            origin_to_tag = numphy.array(
                [
                    [xx, yx, zx, tx + self.bias_x - self.offset_x],
                    [xy, yy, zy, ty + self.bias_y - self.offset_y],
                    [xz, yz, zz, tz + self.bias_z - self.offset_z],
                    [0, 0, 0, 1],
                ]
            ).tolist()

            tags[tag_id] = origin_to_tag  # Store computed transformation

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
