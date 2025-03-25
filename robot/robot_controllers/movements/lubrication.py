from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.robot_controllers.movements.waffle_iron import _check_if_waffle_iron_open
from robot.tools.camera_interface import get_tag_from_camera
from robot.tools.file_manipulation import Jsonreader

import numpy as numphy


def pick_up_lube(bot: Wafflebot, reverse: bool = False)-> bool:
    """
    picks up the lube from the tool station OR 
    places the lube back in the tool station.

    :param reverse: True to put down lube, 
    false to pick up lube.

    :returns bool: True if movement success, False if movement failed. 
    """
    reader = Jsonreader()
    offsets = reader.read("offsets")
    static_objects = reader.read("static_objects")

    if reverse:
        lube_origin = static_objects["lube_toolstation"]
    else:
        lube_origin = get_tag_from_camera("lube")

    # todo call update_offsets() or sumn
    lube_prep_offset = numphy.matrix(offsets["lube_prep"])
    lube_grab_offset = numphy.matrix(offsets["lube_grab"])
    
    lube_prep_pos = lube_origin * lube_prep_offset

    
    # calculate where to go to:
    lube_prep_pos = lube_origin * lube_prep_offset
    # move to prep
    bot.move_old(lube_prep_pos, ["lube"])
    if not reverse:
        bot.gripper.release()
        # update target
        lube_origin = get_tag_from_camera("lube")
        lube_prep_pos = lube_origin * lube_prep_offset
    lube_grab_pos = lube_origin * lube_grab_offset
    # Go to lube
    bot.move_old(lube_grab_pos, ["lube"])
    if reverse:
        bot.gripper.release()
    else:
        bot.gripper.grasp()    
    bot.move_old(lube_prep_pos, ["lube"])

def spray_lube(bot:Wafflebot) -> bool:
    """
    sprays the lube into the waffle iron. 

    :returns bool: True if movement success, False if movement failed. 
    """
    """
    if not _check_if_waffle_iron_open():
        print("robot_movements/lubrication: waffle iron is not open. aborting movement.")
        return False
    """
    reader = Jsonreader()
    offsets = reader.read("offsets")
    
    # Todo change to static objects
    waffle_iron_origin = get_tag_from_camera("waffle_iron")
    front_of_waffle_iron_offset = numphy.matrix(offsets["front_of_waffle_iron"])
    spray_offsets = [
         numphy.matrix(offsets["spray_a"]),
         numphy.matrix(offsets["spray_b"]),
         numphy.matrix(offsets["spray_c"]),
         numphy.matrix(offsets["spray_d"]),
    ]

    front_of_waffle_iron_pos = waffle_iron_origin * front_of_waffle_iron_offset
    
    spray_positions = [0,0,0,0]
    for i in range(len(spray_offsets)):
        spray_positions[i] = waffle_iron_origin * spray_offsets[i]
    
    bot.move_old(front_of_waffle_iron_pos, ["waffle_iron", "lube"])
    
    for i in range(len(spray_offsets)):
        bot.move_old(spray_positions[i], ["waffle_iron", "lube"])
        #spray()
    bot.move_old(front_of_waffle_iron_pos, ["waffle_iron","lube"])

    

    
