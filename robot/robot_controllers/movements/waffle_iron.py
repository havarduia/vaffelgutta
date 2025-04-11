from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
import numpy as numphy


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
    for position in lift_positions:
        bot.move(position, ignore=["waffle_iron", "sticks"])
    bot.gripper.release()


    if reverse:
        bot.move("bottom_of_waffle_iron", ["waffle_iron", "sticks"])
    else:
        bot.move("top_of_waffle_iron",["waffle_iron"])


def insert_sticks(bot: Wafflebot) -> bool:
    if not _check_if_waffle_iron_open:
        print("robot_movements/waffle_iron: waffle iron is not open. Not inserting sticks.")
        return False
    
    reader = Jsonreader()
    offsets         = reader.read("offsets")
    static_objects  = reader.read("static_objects")

    tool_station_origin = get_tag_from_camera("tool_station") 
    waffle_iron_origin  = get_tag_from_camera("waffle_iron")

    front_of_tool_station_offset    =   numphy.matrix(offsets["front_of_tool_station"])
    tool_station_sticks_offset      =   numphy.matrix(static_objects["tool_station_sticks"])
    front_of_waffle_iron_offset     =   numphy.matrix(offsets["front_of_waffle_iron"])
    waffle_iron_sticks_offset       =   numphy.matrix(offsets["waffle_sticks"])
    
    front_of_tool_station_pos       =   tool_station_origin * front_of_tool_station_offset
    tool_station_sticks_pos         =   tool_station_origin * tool_station_sticks_offset
    front_of_waffle_iron_pos        =   waffle_iron_origin  * front_of_waffle_iron_offset 
    waffle_iron_sticks_pos          =   waffle_iron_origin  * waffle_iron_sticks_offset
    
    bot.gripper.release()
    bot.move(front_of_tool_station_pos)
    bot.move(tool_station_sticks_pos,   ["tool_station","sticks"])
    bot.gripper.grasp()
    bot.move(front_of_tool_station_pos, ["tool_station","sticks"])
    bot.move(front_of_waffle_iron_pos,  ["sticks", "waffle_iron"])
    bot.move(waffle_iron_sticks_pos,    ["sticks", "waffle_iron"])
    bot.gripper.release()
    bot.move(front_of_waffle_iron_pos,  ["sticks", "waffle_iron"])
    return True

def take_out_waffle(bot: Wafflebot) -> bool:
    reader  = Jsonreader()
    offsets = reader.read("offsets")
    
    waffle_iron_origin  = get_tag_from_camera("waffle_iron")
    front_of_waffle_iron_offset     =   numphy.matrix(offsets["front_of_waffle_iron"])
    waffle_iron_sticks_offset       =   numphy.matrix(offsets["waffle_sticks"])

    front_of_waffle_iron_pos        =   waffle_iron_origin  * front_of_waffle_iron_offset 
    waffle_iron_sticks_pos          =   waffle_iron_origin  * waffle_iron_sticks_offset

    bot.gripper.release()
    bot.move(front_of_waffle_iron_pos,  ["sticks", "waffle_iron"])
    bot.move(waffle_iron_sticks_pos,    ["sticks", "waffle_iron"])
    bot.gripper.grasp()
    bot.move(front_of_waffle_iron_pos,  ["sticks", "waffle_iron"])
    return True

def take_waffle_off_sticks(bot:Wafflebot):
    reader = Jsonreader()
    static_objects = reader.read("static_objects")
    targets = [
        numphy.matrix(static_objects["pole_a"]),
        numphy.matrix(static_objects["pole_b"]),
        numphy.matrix(static_objects["pole_c"]),
        numphy.matrix(static_objects["pole_d"]),
        numphy.matrix(static_objects["pole_e"]),
    ]
    for target in targets:
        bot.move(target, ["sticks", "pole"])

def put_away_sticks(bot: Wafflebot) -> bool:
    reader = Jsonreader()
    offsets         = reader.read("offsets")
    static_objects  = reader.read("static_objects")
    
    tool_station_origin = get_tag_from_camera("tool_station") 

    front_of_tool_station_offset    =   numphy.matrix(offsets["front_of_tool_station"])
    tool_station_sticks_offset      =   numphy.matrix(static_objects["tool_station_sticks"])
    
    front_of_tool_station_pos       =   tool_station_origin * front_of_tool_station_offset
    tool_station_sticks_pos         =   tool_station_origin * tool_station_sticks_offset

    bot.move(front_of_tool_station_pos, ["sticks"])
    bot.move(tool_station_sticks_pos,   ["tool_station","sticks"])
    bot.gripper.release()
    bot.move(front_of_tool_station_pos, ["tool_station","sticks"])
    return True

def serve_waffle(bot: Wafflebot):
    take_out_waffle(bot)
    take_waffle_off_sticks(bot)
    put_away_sticks(bot)
