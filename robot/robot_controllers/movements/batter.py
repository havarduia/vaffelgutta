from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
from robot.tools.file_manipulation import Jsonreader
from robot.tools.camera_interface import get_tag_from_camera
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.update_tagoffsets import position_from_name 
import numpy as numphy


def put_ladle_in_bowl(bot: Wafflebot):
    """
    Takes the cup and places it at the filling station.
    
    :param is_holding_cup: True if the robot is already holding the cup. 
    False if the robot should attempt to pick up the cup from the ground.

    :returns bool: True if movement success, False if movement failed. 
    """
    reader = Jsonreader()
    positions = reader.read("recordings")

    if bot.automatic_mode:
        top_of_bowl_pos = position_from_name("top_of_bowl")
        inside_bowl_pos = position_from_name("inside_bowl")
        front_of_bowl_pos= positions["front_of_bowl"]["basepose"]
    else:
        top_of_bowl_pos = positions["top_of_bowl"]["joints"]
        inside_bowl_pos = positions["inside_bowl"]["joints"]
        front_of_bowl_pos= positions["front_of_bowl"]["joints"]

    bot.move(front_of_bowl_pos, ignore=["ladle"])
    bot.move(top_of_bowl_pos, ignore=["ladle"])
    bot.move(inside_bowl_pos, ignore= ["ladle", "bowl"])
    bot.gripper.release()
    bot.move(top_of_bowl_pos, ignore=["ladle", "bowl"])
    bot.move(front_of_bowl_pos, ignore=["ladle"])

def thug_shake(bot: Wafflebot):
    poses =(
        get_trajectory_matrix("thug_shake")
        if bot.automatic_mode else
        get_trajectory_joints("thug_shake")
        )
    for pose in poses:
        bot.move(pose, ignore=["ladle"], speed_scaling=3.0) # TODO adjust speed_scaling


def pick_up_ladle(bot: Wafflebot):
    """
    picks up cup from filling station 
    and moves it to the front of the waffle iron.
    """

    bot.gripper.release()
    reader = Jsonreader()
    positions = reader.read("recordings")

    if bot.automatic_mode:
        front_of_bowl_pos= positions["front_of_bowl"]["basepose"]
        inside_bowl_pos = position_from_name("inside_bowl")
        top_of_bowl_pos = position_from_name("top_of_bowl")
    else:
        front_of_bowl_pos= positions["front_of_bowl"]["joints"]
        inside_bowl_pos= positions["inside_bowl"]["joints"]
        top_of_bowl_pos= positions["top_of_bowl"]["joints"]

    bot.move(front_of_bowl_pos)
    bot.move(inside_bowl_pos, ignore= ["ladle", "bowl"])
    bot.gripper.grasp()
    bot.move(top_of_bowl_pos, ignore=["ladle", "bowl"])
    thug_shake(bot)
    bot.move(front_of_bowl_pos, ignore=["ladle"])


def pour_batter(bot: Wafflebot) -> bool:
    """
    pours the batter from a held cup into the waffle iron
    
    :returns bool: True if movement success, False if movement failed. 
    """
    """
    if not _check_if_waffle_iron_open():
        print("robot_movements/batter/pour_batter:\nWaffle iron not open. cancelling movement")
        return False
    """

    reader = Jsonreader()
    offsets = reader.read("offsets") 
    waffle_iron_origin = get_tag_from_camera("waffle_iron")
    
    front_of_waffle_iron_offset = numphy.matrix(offsets["front_of_waffle_iron"])
    pour_offsets = [
        numphy.matrix(offsets["pour_a"]),
        numphy.matrix(offsets["pour_b"]),
        numphy.matrix(offsets["pour_c"]),
        numphy.matrix(offsets["pour_d"])
    ]

    front_of_waffle_iron_pos = waffle_iron_origin * front_of_waffle_iron_offset    
    
    pour_positions = []
    for offset in pour_offsets:
        pour_positions.append(waffle_iron_origin * offset)

    bot.move_old(front_of_waffle_iron_pos, ["cup"])
    for position in pour_positions:
        bot.move_old(position, ["cup", "waffle_iron"])
    bot.move_old(front_of_waffle_iron_pos, ["cup", "waffle_iron"])
    return True
