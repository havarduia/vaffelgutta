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
    (dx,dy,dz) = [out_matrix[:3,i].tolist() for i in range(3)]


    # compute absolute position:
    x,y,z = [(
              xo*dx[i][0]
            + yo*dy[i][0] 
            + zo*dz[i][0]
            )for i in range(3)]
    x+=xt;  y+=yt;  z+=zt 
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
    offset[:3,:3] = current_arm_pos[:3, :3]
    return offset.tolist()

if __name__ == "__main__": 
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