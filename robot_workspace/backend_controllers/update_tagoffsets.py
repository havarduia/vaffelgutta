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
    import_reload(offsets)
    try:
        return getattr(offsets, name)
    except AttributeError:
        print(f"{name} not found in offsets")
        return False


def read_all_offsets():
    import_reload(offsets)
    offset_names = []
    offset_values = []
    for name, _ in inspect.getmembers(offsets):
        if name.startswith("__"): continue
        offset = getattr(offsets,name)
        offset_names.append(name)
        offset_values.append(offset)
      
    return (offset_names, offset_values)


def _write_offset(name,r, theta, z):



    return

def write_offset_matrix(bot:InterbotixManipulatorXS, reference_tag: str):

    tag_matrix = camera_interface.get_tag_from_camera(reference_tag)
    current_pos_matrix = bot.arm.get_ee_pose()
    
    tag_x = tag_matrix[0][3] 
    tag_y = tag_matrix[1][3]
    tag_z = tag_matrix[2][3]
    current_pos_x = current_pos_matrix[0][3] 
    current_pos_y = current_pos_matrix[1][3] 
    current_pos_z = current_pos_matrix[2][3] 

    tag_r = numphy.hypot(tag_x,tag_y)
    tag_theta = numphy.atan2(tag_y,tag_x)
    tag_z = tag_z
    current_pos_r = numphy.hypot(current_pos_x, current_pos_y)
    current_pos_theta = numphy.atan2(current_pos_y, current_pos_x)
    current_pos_z = current_pos_z


    offset_r = tag_r-current_pos_r
    offset_theta = tag_theta - current_pos_theta
    offset_z = tag_z - current_pos_z

    write_offset(tag, )

    return


def update_all_tags():

    tag_l