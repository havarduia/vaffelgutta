from robot_workspace.assets.positions import arm_offsets
from importlib import reload as import_reload
from robot_workspace.assets.Wafflebot import Wafflebot
import numpy as numphy


def _get_lube_tag_from_camera():
    
    # Todo add in camera check:
    # Return 4x4 matrix of waffke iron tag location
    return numphy.matrix(numphy.identity(4)) 

def pick_up_lube(bot: Wafflebot, reverse:bool):
    import_reload(arm_offsets)

    if reverse:
        lube_origin = getattr(arm_positions, "lube_toolstation")
    else:
        lube_origin = _get_lube_tag_from_camera()

    lube_prep_offset = numphy.matrix(getattr(arm_offsets, "lube_prep")) 
    lube_grab_offset = numphy.matrix(getattr(arm_offsets, "lube_grab"))
    
    lube_prep_pos = lube_origin * lube_prep_offset

    
    if reverse:
        lube_prep_pos = numphy.matrix(getattr)
        bot.move(lube_prep_pos, ["lube"])
        bot.move()
    else:
        # calculate where to go to:
        lube_prep_pos = lube_origin * lube_prep_offset
        # move to prep
        bot.move(lube_prep_pos)
        bot.gripper.release()
        # update target
        lube_origin = _get_lube_tag_from_camera()
        lube_prep_pos = lube_origin * lube_prep_offset
        lube_grab_pos = lube_origin * lube_grab_offset
        # Go to lube
        bot.move(lube_grab_pos, ["lube"])
        bot.gripper.grasp()    
        bot.move(lube_prep_pos, ["lube"])


def apply_lube(bot:Wafflebot):
    import_reload(arm_offsets)
    
