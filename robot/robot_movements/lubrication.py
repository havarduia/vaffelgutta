
from robot.backend_controllers.file_manipulation import Jsonreader
from robot.assets.Wafflebot import Wafflebot
from robot.robot_movements.waffle_iron import _check_if_waffle_iron_open
from robot.backend_controllers.camera_interface import get_tag_from_camera
from importlib import reload as import_reload

import numpy as numphy


def pick_up_lube(bot: Wafflebot, reverse:bool = 0):
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
    bot.move(lube_prep_pos, ["lube"])
    if not reverse:
        bot.gripper.release()
        # update target
        lube_origin = get_tag_from_camera("lube")
        lube_prep_pos = lube_origin * lube_prep_offset
    lube_grab_pos = lube_origin * lube_grab_offset
    # Go to lube
    bot.move(lube_grab_pos, ["lube"])
    if reverse:
        bot.gripper.release()
    else:
        bot.gripper.grasp()    
    bot.move(lube_prep_pos, ["lube"])


def apply_lube(bot:Wafflebot):
    if not _check_if_waffle_iron_open():
        print("robot_movements/lubrication: waffle iron is not open. aborting movement.")
        return False
    reader = Jsonreader()
    offsets = reader.read("offsets")
    static_objects = reader.read("static_objects") 
    
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
    
    bot.move(front_of_waffle_iron_pos, ["waffle_iron", "lube"])
    
    for i in range(len(spray_offsets)):
        bot.move(spray_positions[i], ["waffle_iron", "lube"])
        #spray()
    bot.move(front_of_waffle_iron_pos, ["waffle_iron","lube"])

    

    
