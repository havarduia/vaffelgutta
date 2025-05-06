from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
from robot.tools.file_manipulation import Jsonreader
from robot.tools.camera_interface import get_tag_from_camera
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.update_tagoffsets import position_from_name 
import numpy as numphy


def put_ladle_in_bowl(bot: Wafflebot):
    """
    Takes the ladle and places it in the bowl
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
    bot.move("thug_shake_1", ignore=["ladle"], speed_scaling=6.0) # TODO adjust speed_scaling
    bot.move("thug_shake_2", ignore=["ladle"], speed_scaling=6.0) # TODO adjust speed_scaling
    bot.move("thug_shake_3", ignore=["ladle"], speed_scaling=6.0) # TODO adjust speed_scaling
    bot.move("thug_shake_4", ignore=["ladle"], speed_scaling=6.0) # TODO adjust speed_scaling


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
    bot.move(top_of_bowl_pos, ignore=["ladle", "bowl"])
    bot.move(front_of_bowl_pos, ignore=["ladle"])


def pour_batter(bot: Wafflebot): 
    """
    pours the batter from a held cup into the waffle iron
    """
    reader = Jsonreader()
    positions = reader.read("recordings")
    
    bot.move("front_of_waffle_iron_ladle", ignore=["ladle"])

    pose_type = "basepose" if bot.automatic_mode else "joints"
    pour_positions = [
        positions["pour_1"][pose_type],
        positions["pour_2"][pose_type],
        positions["pour_3"][pose_type],
        positions["pour_4"][pose_type],
    ]
    for pose in pour_positions:
        bot.move(pose, ignore=["ladle","waffle_iron"])

    bot.move("front_of_waffle_iron_ladle", ignore=["ladle","waffle_iron"])

