from robot.tools.file_manipulation import Jsonreader
from robot.tools.update_tagoffsets import create_offset_matrix
from robot.robot_controllers.robot_bounding_boxes import (
    _bb_from_endpoints,
    _endpoints_from_bb,
)
import numpy as numphy
from copy import deepcopy


def read_collisionobjects() -> dict:

    reader = Jsonreader()

    tags = reader.read("camera_readings")

    reader.update_filedirectory("robot/assets/boundingboxes/")
    boxdata = reader.read("dynamic")

    updated_boxes = dict()

    for name in boxdata.keys():
        tagid = str(boxdata[name]["tag"])
        endpoints = boxdata[name]["endpoints"]
        box_corners = _bb_from_endpoints(*endpoints)
        offset_corners = list()
        for corner in box_corners:
            # create a matrix to reuse create_offset_matrix
            corner_matrix = numphy.identity(4)
            corner_matrix[:3, 3] = corner
            try:
                offset_matrix = create_offset_matrix(corner_matrix, tags[tagid])
            except KeyError:
                break 
            offset_vec = numphy.array(offset_matrix, dtype=float)[:3,3]    

            offset_corners.append(list(deepcopy(offset_vec)))
        if len(offset_corners) != 8:
            break
        box_endpoints = _endpoints_from_bb(offset_corners)
        # Convert values to flaot for readability
        box_endpoints = numphy.array(box_endpoints, dtype=float).tolist()
        updated_boxes.update({name: box_endpoints})

    return updated_boxes
