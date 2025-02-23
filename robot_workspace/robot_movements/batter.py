from robot_workspace.assets.positions import tools, positions, offsets
from robot_workspace.backend_controllers.camera_interface import get_tag_from_camera
from robot_workspace.assets.Wafflebot import Wafflebot
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
    print("robot_movements/batter.py: pickup_fallen_cup is not implemented")
    raise NotImplementedError
    # Todo in future implement a function to pick up a cup that has fallen on its side 

def place_cup_at_filling_station(bot: Wafflebot):
    import_reload(offsets)
    import_reload(tools)
    import_reload(positions)

    filling_station_origin  = get_tag_from_camera("filling_station")
    cup_origin              = get_tag_from_camera("cup")

    cam_pos                 = numphy.matrix(getattr(positions, "tool_camera"))
    filling_station_offset  = numphy.matrix(getattr(offsets, "filling_station"))
    cup_offset              = numphy.matrix(getattr(offsets, "cup"))

    filling_station_pos = filling_station_origin * filling_station_offset
    cup_pos             =             cup_origin * cup_offset
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
        raise Exception("Bot is not holding cup - Manual intervention needed!!")
        return False
    bot.move(filling_station_pos, ["cup", "filling_station"])
    bot.gripper.release()
    bot.move("home", ["cup", "filling_station"])
    return True

def fill_cup(bot: Wafflebot):
    # Interface with Aleksanders mystery box hardware
    print("robot_movements/batter.py:\npickup_fallen_cup is not implemented")
    raise NotImplementedError

    
def pick_up_cup_from_filling_station():
    ...

def pour_batter():
    ...
