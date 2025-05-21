# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot.tools import camera_interface
from robot.tools.file_manipulation import Jsonreader
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy


def create_offset_matrix(current_arm_pos: list[list[float]], tag: list[list[float]])->list[list[float]]:
    """
    returns the offset matrix for a position to the arm from a tag  
    """
    current_arm_pos = numphy.matrix(current_arm_pos)
    tag = numphy.matrix(tag)
    print(tag)
    offset = numphy.linalg.inv(tag)*current_arm_pos
    return offset.tolist()


def abs_position_from_offset(reference_tag, offset):
    #reference tag and offset are 4x4 matrices expressed as list of list
    tag     = numphy.matrix(reference_tag)
    offset  = numphy.matrix(offset)
    out_matrix = tag * offset # initialize rotation, but xyz is off

    return out_matrix.tolist()

def position_from_name(name: str):
    reader = Jsonreader()
    tags = reader.read("camera_readings")
    positions = reader.read("recordings")
    offset = positions[name]["offset"]
    tagid = positions[name]["tag"]
    tag = tags[str(tagid)]
    pos = abs_position_from_offset(tag, offset)
    return pos

    

    


if __name__ == "__main__": 
    # test script:
    tag = ([
    [ 0.9638844,  -0.26469881, -0.02934978,  0.30423656],
    [ 0.26467165,  0.96432626, -0.00487716, -0.22009233],
    [ 0.02959374, -0.00306704,  0.9995573 ,  0.48485223],
    [ 0.       ,   0.,          0.       ,   1.        ] 
          ])
    
    offset = ([
        [0,1,0,1],
        [1,0,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
    print(numphy.array(
       abs_position_from_offset(tag, offset)
    ))
