from robot.tools.file_manipulation import Jsonreader
from camera.misc import print_blue, print_error, ConfigLoader
import numpy as numphy


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
        self.transformations = self.aruco.estimate_pose()
        self.origin_id = config_loader.get("origin_id")
        self.offset_x = config_loader.get("offset_x")
        self.offset_y = config_loader.get("offset_y")
        self.offset_z = config_loader.get("offset_z")
        self.bias_x = config_loader.get("bias_x")
        self.bias_y = config_loader.get("bias_y")
        self.bias_z = config_loader.get("bias_z")

    def transformation_origin_to_tag(self):

        if self.origin_id not in self.transformations:
            print_error(f"Error: Marker {self.origin_id} not detected! 👺")
            return {}

        origin = self.transformations[self.origin_id]
        origin_inv = numphy.linalg.inv(origin)
        tags = {}  # Dictionary to store transformations

        for tag_id, transformation in self.transformations.items():
            tag_id = int(tag_id)
            if tag_id == self.origin_id:
                continue

            origin_to_tag = numphy.dot(origin_inv, transformation)

            # Reorder the transformation matrix elements
            xx = origin_to_tag[1][1]
            xy = -origin_to_tag[0][1]
            xz = origin_to_tag[2][1]

            yx = -origin_to_tag[1][0]
            yy = origin_to_tag[0][0]
            yz = -origin_to_tag[2][0]

            zx = origin_to_tag[1][2]
            zy = -origin_to_tag[0][2]
            zz = origin_to_tag[2][2]

            tx = origin_to_tag[1][3]
            ty = -origin_to_tag[0][3]
            tz = origin_to_tag[2][3]

            origin_to_tag = numphy.array(
                [
                    [xx, yx, zx, tx + self.bias_x - self.offset_x],
                    [xy, yy, zy, ty + self.bias_y - self.offset_y],
                    [xz, yz, zz, tz + self.bias_z - self.offset_z],
                    [0, 0, 0, 1],
                ]
            ).tolist()

            tags[tag_id] = origin_to_tag  # Store result

        return tags

    def init_tags(self, tags: str | int):
        all_tags = []
        for tag in tags:
            all_tags.append(str(tag))
        return all_tags

    def start(self, *allowed_tags: str | int) -> list[str]:
        config_loader = ConfigLoader()
        reader = Jsonreader()
        self.transformations = self.aruco.estimate_pose()
        tags = self.transformation_origin_to_tag()

        allowed_tags = self.init_tags(allowed_tags)
        for id in tags.keys():
            id = str(id)

        reader.write("camera_readings", tags)
        data = reader.read("camera_readings")

        for key in data.keys():
            print(key)
            if not key in allowed_tags:
                print(f"removed hallucinated tag, id: {key}")
                reader.pop("camera_readings", key)
