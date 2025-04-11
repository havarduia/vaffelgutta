from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.file_manipulation import Jsonreader
from robot.tools.update_tagoffsets import position_from_name


import numpy as numphy


def pick_up_lube(bot: Wafflebot, reverse: bool = False):
    """
    picks up the lube from the tool station OR 
    places the lube back in the tool station.

    :param reverse: True to put down lube, 
    false to pick up lube.

    :returns bool: True if movement success, False if movement failed. 
    """
    reader = Jsonreader()
    positions = reader.read("recordings")

    #ensure gripper is open for the movement 
    if not reverse:
        bot.gripper.release()
    # move to prep
    if bot.automatic_mode:
        top_of_lube_pos = position_from_name("top_of_lube") 
    else:
        top_of_lube_pos = positions["top_of_lube"]["joints"]

    bot.move(top_of_lube_pos, ["lube", "toolstation"])

    # Go to lube
    if bot.automatic_mode:
        lube_toolstation_pos = position_from_name("lube_toolstation") 
    else:
        lube_toolstation_pos = positions["lube_toolstation"]["joints"]

    bot.move(lube_toolstation_pos, ["lube", "toolstation"])

    if reverse:
        bot.gripper.release()
    else:
        bot.gripper.grasp()    

    bot.move(top_of_lube_pos, ["lube", "toolstation"])

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

    

    
