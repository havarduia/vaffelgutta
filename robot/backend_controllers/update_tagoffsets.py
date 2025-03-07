# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.backend_controllers import camera_interface
from robot_workspace.backend_controllers.file_manipulation import Jsonreader
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy

def read_single_offset(name:str):
    """
    Returns the specified offset matrix
    """
    reader = Jsonreader()
    offsets = reader.read("offsets")
    try:
        return offsets[name]
    except KeyError:
        print(f"{name} not found in offsets")
        return False

    
def abs_position_from_offset(reference_tag, offset):
    #reference tag and offset are 4x4 matrices expressed as list of list
    tag     = numphy.matrix(reference_tag)
    offset  = numphy.matrix(offset)
    out_matrix = tag * offset # initialize rotation, but xyz is off
    # Naming convention: o-ffset, t-ag, a-bsolute pos 
    #compute x-y-z 
    (xt,yt,zt) = [tag   [i,3]   for i in range(3)]
    (xo,yo,zo) = [offset[i,3]   for i in range(3)]
    (dx,dy,dz) = [tag   [:3,i]  for i in range(3)]
    # compute absolute position:
    x,y,z = [(
              xt*dx[i]
            + yt*dy[i] 
            + zt*dz[i]
            )[0,0] # squeeze out item from 2d array
            for i in range(3)]
    x+=xo;  y+=yo;  z+=zo 
    # Collect the terms into a single output matrix
    out_matrix[0,3] = x
    out_matrix[1,3] = y
    out_matrix[2,3] = z
    return out_matrix.tolist() 


def create_offset_matrix(current_arm_pos: list[list[float]], tag: list[list[float]])->list[list[float]]:
    """
    returns the offset matrix for a position to fo from a tag to a string 
    """
    current_arm_pos = numphy.matrix(current_arm_pos)
    tag = numphy.matrix(tag)
    offset = numphy.linalg.inv(tag)*current_arm_pos
    return offset.tolist()
