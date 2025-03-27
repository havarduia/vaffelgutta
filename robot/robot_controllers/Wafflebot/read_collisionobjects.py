from robot.tools.file_manipulation import Jsonreader
from robot.tools.update_tagoffsets import create_offset_matrix
from robot.robot_controllers.robot_bounding_boxes import (
    _bb_from_endpoints,
    _endpoints_from_bb,
)
import numpy as numphy
from copy import deepcopy


def read_collisionobjects() -> None:

    reader = Jsonreader()

    tags = reader.read("camera_readings")

    reader.update_filedirectory("robot/assets/boundingboxes/")
    boxdata = reader.read("dynamic_origins")

    updated_boxes = dict()

    for name in boxdata.keys():
        tagid = boxdata[name]["tag"]
        endpoints = boxdata[name]["endpoints"]

        box_corners = _bb_from_endpoints(*endpoints)
        offset_corners = list()
        for corner in box_corners:
            # create a matrix to reuse create_offset_matrix
            corner_matrix = numphy.identity(4)
            corner_matrix[:3, 3] = corner
            offset_matrix = create_offset_matrix(corner_matrix, tags[str(tagid)])
            
            offset_vec = numphy.array(offset_matrix)[:3,3]
            
            offset_corners.append(list(deepcopy(offset_vec)))

        box_endpoints = _endpoints_from_bb(offset_corners)
        updated_boxes.update({name: box_endpoints})

    reader.clear("dynamic")
    reader.write("dynamic", updated_boxes)

read_colisionobjects()