from robot_workspace.backend_controllers.file_manipulation import Jsonreader
from robot_workspace.backend_controllers.camera_interface import get_tag_from_camera
from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.robot_movements.waffle_iron import _check_if_waffle_iron_open
from importlib import reload as import_reload
import numpy as numphy

def _test_if_cup_is_upright(cup_origin, cup_offset, cup_pos):
    # Use tag detection to determine orientation...
    raise NotImplementedError

def _check_if_holding_cup():
    # Todo: use a camera check to see if cup is within a distance threshold
    # of the expected position.
    # Expected output:
    # True if bot is holding cup
    # True (false positive) if cup is laying very close by
    # False if cup not detected or not within threshold of distance from expected (once measured)
    return True

def pickup_fallen_cup(bot: Wafflebot):
    print("robot_movements/batter: pickup_fallen_cup is not implemented")
    raise NotImplementedError
    # Todo in future implement a function to pick up a cup that has fallen on its side 

def place_cup_at_filling_station(bot: Wafflebot, is_holding_cup:bool = False):
    reader = Jsonreader()
    static_objects  = reader.read("static_objects")
    offsets         = reader.read("offsets")

    filling_station_origin          = get_tag_from_camera("filling_station")
    cup_origin                      = get_tag_from_camera("cup")
    
    cam_pos                         = numphy.matrix(static_objects["cam"])

    front_of_filling_station_offset = numphy.matrix(offsets["front_of_waffle_iron"])
    filling_station_offset          = numphy.matrix(offsets["filling_station"])
    cup_offset                      = numphy.matrix(offsets["cup"])

    front_of_filling_station_pos    = filling_station_origin * front_of_filling_station_offset
    filling_station_pos             = filling_station_origin * filling_station_offset
    cup_pos                         =             cup_origin * cup_offset
    if not is_holding_cup:
        try:
            if not _test_if_cup_is_upright(cup_origin, cup_offset, cup_pos):
                pickup_fallen_cup(bot)
        except NotImplementedError:
            print ("robot_movements/batter.py:\nfunction not made yet. Assuming success.")
    
        bot.gripper.release()
        bot.move(cup_pos, ["cup", "filling_station", "waffle_iron"])
        bot.gripper.grasp()
    
        bot.move(cam_pos,["cup", "filling_station"])
        if not _check_if_holding_cup():
            raise Exception("Bot failed to grasp the cup.\nMANUAL INTERVENTION NEEDED!!")
            return False
    
    bot.move(front_of_filling_station_pos, ["cup"])
    bot.move(filling_station_pos, ["cup", "filling_station"])
    bot.gripper.release()
    bot.move(front_of_filling_station_pos, ["cup", "filling_station"])
    return True

def fill_cup(bot: Wafflebot):
    # Interface with Aleksanders mystery box hardware
    print("robot_movements/batter.py:\npickup_fallen_cup is not implemented")
    raise NotImplementedError

    
def pick_up_cup_from_filling_station(bot: Wafflebot):
    reader = Jsonreader()
    offsets = reader.read("offsets")

    cup_origin                      = get_tag_from_camera("cup") 
    waffle_iron_origin              = get_tag_from_camera("waffle_iron")
    filling_station_origin          = get_tag_from_camera("filling_station")

    cup_offset                      = numphy.matrix(offsets["cup"])
    front_of_waffle_iron_offset     = numphy.matrix(offsets["front_of_waffle_iron"])
    front_of_filling_station_offset = numphy.matrix(offsets["front_of_filling_station"])

    cup_pos                         = cup_origin * cup_offset
    front_of_waffle_iron_pos        = waffle_iron_origin * front_of_waffle_iron_offset
    front_of_filling_station_pos    = filling_station_origin * front_of_filling_station_offset

    bot.gripper.release() 
    bot.move(cup_pos, ["cup", "filling_station"])
    bot.gripper.grasp()
    bot.move(front_of_filling_station_pos, ["cup", "filling_station"])
    bot.move(front_of_waffle_iron_pos, ["cup"])

def pour_batter(bot: Wafflebot):
    if not _check_if_waffle_iron_open():
        print("robot_movements/batter/pour_batter:\nWaffle iron not open. cancelling movement")
    
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

    bot.move(front_of_waffle_iron_pos, ["cup"])
    for position in pour_positions:
        bot.move(position, ["cup", "waffle_iron"])
    bot.move(front_of_waffle_iron_pos, ["cup", "waffle_iron"])