from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets.positions import arm_offsets
from importlib import reload as import_reload
import numpy as numphy


def _check_if_waffle_iron_open():

    # Todo add in camera checks  
    # Return True if iron is open,
    # return false if it is closed (not open)
    return False
def _get_waffle_iron_tag_from_camera():
    
    # Todo add in camera check:
    # Return 4x4 matrix of waffke iron tag location
    return numphy.identity(4) 

def _get_waffle_iron_lift_offsets(movement_is_up: bool):
    waypoint_count = 1
    
    waypoints = []
    for i in range (waypoint_count):
        if movement_is_up:
            waypoints.append(numphy.matrix(getattr(arm_offsets, f"waffle_iron_open_{i}")))
        else:
            waypoints.append(numphy.matrix(getattr(arm_offsets, f"waffle_iron_open_{waypoint_count-i}")))
    return waypoints

def openwaffleiron(bot: Wafflebot, reverse:bool):
    if _check_if_waffle_iron_open():
        if not reverse:
            print("Robot movements->waffleiron:")
            print("Waffle iron already open. Aborting movement.")
            return False
    elif reverse:
        print("Robot movements->waffleiron:")
        print("Waffle iron already closed. Aborting movement.")
        return False

    import_reload(arm_offsets)
    
    waffle_iron_pos         = numphy.matrix(_get_waffle_iron_tag_from_camera()       )
    front_of_iron_offset    = numphy.matrix( getattr(   arm_offsets, "front_of_iron"))    
    lift_offsets            = numphy.matrix(_get_waffle_iron_lift_offsets(True)           )    
    top_of_iron_offset      = numphy.matrix( getattr(   arm_offsets, "top_of_iron")  )

    front_of_iron_pos       =  waffle_iron_pos * front_of_iron_offset 
    lift_positions          = [waffle_iron_pos * offset for offset in lift_offsets]
    top_of_iron_pos         =  waffle_iron_pos * top_of_iron_offset

    lift_positions_count = len(lift_positions)-1

    bot.release()
    if not reverse:
        bot.move(front_of_iron_pos, ["Waffleiron"])
        bot.move(lift_positions[0],["waffleiron"])
        bot.gripper.grasp()
        for i in range(lift_positions_count):
            bot.move(lift_positions[i],["Waffleiron"])
        bot.gripper.release()
        bot.move(top_of_iron_pos,["Waffleiron"])
    else:
        bot.move(top_of_iron_pos,["Waffleiron"])
        bot.move(lift_positions[lift_positions_count],["waffleiron"])
        bot.gripper.grasp()
        for i in range(lift_positions_count):
            bot.move(lift_positions[lift_positions_count-i],["Waffleiron"])
        bot.gripper.release()
        bot.move(front_of_iron_pos, ["Waffleiron"])
