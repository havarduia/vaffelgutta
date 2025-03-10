# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
from robot.assets.Wafflebot import Wafflebot
from robot.backend_controllers.file_manipulation import Jsonreader
from robot.backend_controllers.camera_interface import get_tag_from_camera
from importlib import reload as import_reload
import numpy as numphy


def _check_if_waffle_iron_open():

    # Todo add in camera checks  
    # Return True if iron is open,
    # return false if it is closed (not open)
    return False

def _get_waffle_iron_lift_offsets(movement_is_up: bool = True):
    waypoint_count = 4
    waypoints = []
    char_a_ind = 97
    reader = Jsonreader()
    offsets = reader.read("offsets")
    for i in range (waypoint_count):
        if movement_is_up:
            waypoints.append(numphy.matrix(offsets[f"waffle_iron_open_{chr(char_a_ind+i)}"]))
        else:
            waypoints.append(numphy.matrix(offsets[f"waffle_iron_open_{chr(char_a_ind+waypoint_count-i)}"]))
    return waypoints

def open_waffle_iron(bot: Wafflebot, reverse:bool = False):
    if _check_if_waffle_iron_open():
        if not reverse:
            print("Robot movements/waffle_iron:")
            print("Waffle iron already open. Not opening iron.")
            return False
    elif reverse:
        print("Robot movements/waffle_iron:")
        print("Waffle iron already closed. Not closing iron.")
        return False

    reader = Jsonreader()
    offsets = reader.read("offsets")
    waffle_iron_origin      = get_tag_from_camera("waffle_iron")

    front_of_iron_offset    = numphy.matrix(offsets["front_of_waffle_iron"]) 
    top_of_iron_offset      = numphy.matrix(offsets["top_of_waffle_iron"])
    lift_offsets            =_get_waffle_iron_lift_offsets(movement_is_up=True)    

    front_of_iron_pos       =  waffle_iron_origin * front_of_iron_offset 
    top_of_iron_pos         =  waffle_iron_origin * top_of_iron_offset
    lift_positions          = [waffle_iron_origin * offset for offset in lift_offsets]

    lift_positions_count = len(lift_positions)-1

    bot.gripper.release()
    if reverse:
        bot.move(top_of_iron_pos,   ["waffle_iron"])
    else:
        bot.move(front_of_iron_pos, ["waffle_iron", "sticks"])
    bot.move    (lift_positions[0], ["waffle_iron", "sticks"])
    bot.gripper.grasp()
    for i in range(lift_positions_count):
        bot.move(lift_positions[i], ["waffle_iron", "sticks"])
    bot.gripper.release()
    if reverse:
        bot.move(front_of_iron_pos, ["waffle_iron", "sticks"])
    else:
        bot.move(top_of_iron_pos,   ["waffle_iron"])
    
def insert_sticks(bot: Wafflebot):
    if not _check_if_waffle_iron_open:
        print("robot_movements/waffle_iron: waffle iron is not open. Not inserting sticks.")
        return False
    
    reader = Jsonreader()
    offsets         = reader.read("offsets")
    static_objects  = reader.read("static_objects")

    tool_station_origin = get_tag_from_camera("tool_station") 
    waffle_iron_origin  = get_tag_from_camera("waffle_iron")

    front_of_tool_station_offset    =   numphy.matrix(offsets["front_of_tool_station"])
    tool_station_sticks_offset      =   numphy.matrix(static_objects[  "tool_station_sticks"])
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

def take_out_waffle(bot: Wafflebot):
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

def put_away_sticks(bot: Wafflebot):
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


def take_out_and_serve_waffle(bot: Wafflebot):
    take_out_waffle(bot)
    take_waffle_off_sticks(bot)
    put_away_sticks(bot)
