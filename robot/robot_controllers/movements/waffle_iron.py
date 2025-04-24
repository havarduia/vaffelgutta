from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
import numpy as numphy

from robot.tools.update_tagoffsets import abs_position_from_offset, position_from_name


def open_waffle_iron(bot: Wafflebot, reverse:bool = False):
    """Opens the waffle iron"""
    
    bot.gripper.release()
    if reverse:
        bot.move("top_of_waffle_iron", ignore=["waffle_iron"])
    else:
        bot.move("bottom_of_waffle_iron",ignore=["waffle_iron", "sticks"])

    if bot.automatic_mode:
        lift_positions = get_trajectory_matrix("waffle_iron_open")
    else:
        lift_positions = get_trajectory_joints("waffle_iron_open")

    if reverse:
        lift_positions.reverse()
    bot.move(lift_positions[0], ignore=["waffle_iron", "sticks"])
    bot.gripper.grasp()
    bot.move("waffle_iron_open", ignore=["waffle_iron", "sticks"])
    bot.gripper.release()


    if reverse:
        bot.move("bottom_of_waffle_iron", ["waffle_iron", "sticks"])
    else:
        bot.move("top_of_waffle_iron",["waffle_iron"])


def insert_sticks(bot: Wafflebot):
    
    reader = Jsonreader()
    positions = reader.read("recordings")

    movement_type = "basepose" if bot.automatic_mode else "joints"
    front_of_waffle_iron_pos = positions["front_of_waffle_iron"][movement_type]
    waffle_iron_sticks_pos = positions["waffle_iron_sticks"][movement_type]
    
    bot.move(front_of_waffle_iron_pos, ignore=["sticks"])
    bot.move(waffle_iron_sticks_pos, ignore=["sticks", "waffle_iron"])
    bot.gripper.release()
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"])


def take_out_waffle(bot: Wafflebot):
    reader  = Jsonreader()
    positions = reader.read("recordings")

    # innitialize positions:
    movement_type = "basepose" if bot.automatic_mode else "joints"
    front_of_waffle_iron_pos = positions["front_of_waffle_iron"][movement_type]
    waffle_iron_sticks_pos = positions["waffle_iron_sticks"][movement_type]
    
    #movement sequence:
    bot.gripper.release()
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"])
    bot.move(waffle_iron_sticks_pos,    ignore=["sticks", "waffle_iron"])
    bot.gripper.grasp()
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"])

def take_waffle_off_sticks(bot:Wafflebot):
    reader = Jsonreader()
    positions = reader.read("recordings")

    targets = [
        positions["pole_a"],
        positions["pole_b"],
        positions["pole_c"],
        positions["pole_d"],
        positions["pole_e"],
    ]

    if bot.automatic_mode:
        tags = reader.read("camera_readings") 
        tagid = targets[0]["tag"] #all these tags are the same. using first.
        tag = tags[tagid]
        for target in targets:
            offset = target["offset"]
            pose = abs_position_from_offset(tag, offset)
            bot.move(pose, ignore=["sticks", "pole"]) 
    else:
        [bot.move(target["joints"], ignore=["sticks", "pole"]) for target in targets]


