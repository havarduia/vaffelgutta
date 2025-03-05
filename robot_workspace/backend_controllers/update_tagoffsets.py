# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.positions import offsets
from robot_workspace.backend_controllers import camera_interface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy
from importlib import reload as import_reload
import inspect

def read_single_offset(name:str):
    """
    Returns the specified offset matrix
    """
    import_reload(offsets)
    try:
        return getattr(offsets, name)
    except AttributeError:
        print(f"{name} not found in offsets")
        return False


def read_all_offsets():
    """
    Read the offsets from all tags.
    Returns two lists with name - matrix correlation by index 
    """
    import_reload(offsets)
    offset_names = []
    offset_matrices = []
    for name, matrix in inspect.getmembers(offsets):
        if name.startswith("__"): continue
        offset_names.append(name)
        offset_matrices.append(matrix)      
    return (offset_names, offset_matrices)


def _get_index_of_name(name, names):
    """
    returns the index of a name from a list of names.
    if the name is not in the list of names, return -1.
    """
    for i in range(len(names)-1):
        if name == names[i]:
            return i
    return -1

def write_offset(name :str, matrix :list[list[float]]) -> None:
    """
    writes a matrix to the offsets file.
    
    :param name: the name to write to file
    :param matrix, the 4x4 matrix to write 
    """
    names, matrices = read_all_offsets()
    ind = _get_index_of_name(name, names)
    if ind != -1:
        names.pop(ind)
        matrices.pop(ind)
    names.append(name)
    matrices.append(matrix)

    with open(offsets,"w") as file: 
        for name, matrix in zip(names, matrices):
            file.write(f"{name} = {matrix}")
    return

    
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


def create_offset_matrix(bot: InterbotixManipulatorXS, tag: any)->list[list[float]]:
    print("create_offset_matrix not made yet")
    raise NotImplementedError()
    if isinstance (tag, str):
        #tag = read tag
        ...
    # read offset from file
    return abs_position_from_offset(tag, offset)



