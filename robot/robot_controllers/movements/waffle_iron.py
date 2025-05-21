from time import sleep
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
import numpy as numphy

from robot.tools.update_tagoffsets import abs_position_from_offset, position_from_name


def open_waffle_iron(bot: Wafflebot, reverse:bool = False):
    """Opens the waffle iron"""
    
    bot.release()
    if reverse:
        bot.move("top_of_waffle_iron", ignore=["waffle_iron"])
    else:
        bot.move("bottom_of_waffle_iron",ignore=["waffle_iron", "sticks"])

 
    lift_positions = get_trajectory_joints("waffle_iron_open")

    if reverse:
        lift_positions.reverse()

    bot.arm.set_joint_positions(lift_positions[0], blocking = True, moving_time=2.0)
    bot.grasp()
    wp_length = len(lift_positions)
    for pos in lift_positions:
        bot.arm.set_joint_positions(pos, moving_time=0.2, blocking=False)
        sleep(2.0/(wp_length))
    bot.arm.set_joint_positions(lift_positions[-1], blocking = True, moving_time=2.0)
    bot.release()


    if reverse:
        bot.move("bottom_of_waffle_iron", ["waffle_iron", "sticks"])
    else:
        bot.move("top_of_waffle_iron",["waffle_iron"])


def insert_sticks(bot: Wafflebot):
    
    reader = Jsonreader()
    positions = reader.read("recordings")

    movement_type = "basepose" if bot.automatic_mode else "joints"
    pole_1_pos  = positions["pole_1"][movement_type]
    pole_2_pos  = positions["pole_2"][movement_type]
    front_of_waffle_iron_pos = positions["front_of_waffle_iron"][movement_type]
    front_of_waffle_iron_ladle_pos = positions["front_of_waffle_iron_ladle"][movement_type]
    front_of_waffle_iron_sticks_pos = positions["waffle_iron_sticks_prepare"][movement_type]
    waffle_iron_sticks_pos = positions["waffle_iron_sticks"][movement_type]


    bot.move(pole_2_pos,ignore = ["sticks","pole"],speed_scaling = 0.4)
    bot.move(pole_1_pos,ignore = ["sticks","pole"],speed_scaling = 0.4 )

    bot.move(front_of_waffle_iron_ladle_pos,ignore = ["sticks","waffle_iron"],speed_scaling = 0.4)
    bot.move(front_of_waffle_iron_pos,ignore = ["sticks","waffle_iron"],speed_scaling = 0.6)
    bot.move(front_of_waffle_iron_sticks_pos, ignore=["sticks", "waffle_iron"],speed_scaling = 0.6)

    bot.move(waffle_iron_sticks_pos, ignore=["sticks", "waffle_iron"],speed_scaling = 0.4)
    bot.release()

    bot.move(front_of_waffle_iron_sticks_pos, ignore=["sticks", "waffle_iron"],speed_scaling = 0.7)
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"])


def take_out_waffle(bot: Wafflebot):
    reader  = Jsonreader()
    positions = reader.read("recordings")

    # initialize positions:
    movement_type = "basepose" if bot.automatic_mode else "joints"
    front_of_waffle_iron_pos = positions["front_of_waffle_iron"][movement_type]
    waffle_iron_sticks_prep_pos = positions["waffle_iron_sticks_prepare"][movement_type]
    waffle_iron_sticks_pos = positions["waffle_iron_sticks"][movement_type]
    
    #movement sequence:
    bot.release()
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"])
    bot.move(waffle_iron_sticks_prep_pos,ignore=["sticks", "waffle_iron"])
    bot.move(waffle_iron_sticks_pos,    ignore=["sticks", "waffle_iron"])
    bot.grasp()
    bot.move(waffle_iron_sticks_prep_pos,ignore=["sticks", "waffle_iron"])
    bot.move(front_of_waffle_iron_pos,  ignore=["sticks", "waffle_iron"],speed_scaling = 0.8)

def take_waffle_off_sticks(bot:Wafflebot):
    reader = Jsonreader()
    positions = reader.read("recordings")

    targets = [
        positions["pole_1"],
        positions["pole_2"],
        positions["pole_3"],
        positions["pole_4"],
        #positions["pole_5"],
    ]

    if bot.automatic_mode:
        for target in targets: 
            bot.move(target["basepose"], ignore=["sticks", "pole"],speed_scaling = 0.5) 
    else:
        [bot.move(target["joints"], ignore=["sticks", "pole"],speed_scaling = 0.5) for target in targets]


