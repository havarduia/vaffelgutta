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
        bot.release()
    # move to prep
    if bot.automatic_mode:
        if reverse:
            top_of_lube_pos = positions["top_of_lube"]["basepose"]
        else:
            top_of_lube_pos = position_from_name("top_of_lube") 
    else:
        top_of_lube_pos = positions["top_of_lube"]["joints"]

    bot.move(top_of_lube_pos, ignore=["lube", "toolstation"])

    # Go to lube

    if bot.automatic_mode:
        lube_toolstation_pos = position_from_name("lube_toolstation") 
    else:
        lube_toolstation_pos = positions["lube_toolstation"]["joints"]

    bot.move(lube_toolstation_pos, ["lube", "toolstation"])

    if reverse:
        bot.release()
    else:
        bot.grasp()    


def spray():
    raise NotImplementedError("The spray hardware does not exist yet.")

def spray_lube(bot:Wafflebot): 
    """
    sprays the lube into the waffle iron. 
    """
    reader = Jsonreader()
    positions = reader.read("recordings")
    
    pose_type = "basepose" if bot.automatic_mode else "joints"
    spray_positions = [
         positions["spray_1"][pose_type],
         positions["spray_2"][pose_type],
         positions["spray_3"][pose_type],
         positions["spray_4"][pose_type],
    ]

    #move to the front of the waffle iron
    front_of_waffle_iron_pos = positions["front_of_waffle_iron"][pose_type]
    bot.move(front_of_waffle_iron_pos, ["waffle_iron","lube"])
    
    #apply spray
    for pos in spray_positions:
        bot.move(pos, ignore=["waffle_iron", "lube"])
        try:
            spray()
        except NotImplementedError:
            print("IM SPRAYING AAHHHHH")
    
    #move to the front of the waffle iron
    bot.move(front_of_waffle_iron_pos, ["waffle_iron","lube"])

    

    
