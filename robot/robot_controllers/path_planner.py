from typing import Optional, Literal
from robot.robot_controllers.safety_functions import check_collisions, fix_joint_limits
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot.tools.file_manipulation import Jsonreader
import numpy as numphy
from typing import Literal
from robot.tools.file_manipulation import Jsonreader

def calculate_biggest_joint(joints):    
    biggest_joint = 0
    for joint in joints:
        joint = abs(joint)
        if joint > biggest_joint:
            biggest_joint = joint
    return biggest_joint

def test_endpoint(stop, ignore: list, debug_print):    
    
    boom, botbox, itembox = check_collisions(stop, ignore)
    if boom and debug_print:
        print(f"invalid endpoint - {botbox} collides with {itembox}")
    return boom

def list_sum(list_items, values):
    items = []
    # ensure list compatibility and check if the value is a single number    
    values = [values] if not isinstance(values, list) else values 
    single_item = True if len(values) == 1 else False 
 
    count = len(list_items)
    for i in range(count):
        if single_item:
            values.append(values[0])
        items.append(list_items[i]+values[i]) # <-- bread and butter of this whole operation chief
    return items

def list_multiply(list_items, values):
    items = []
    # ensure list compatibility and check if the value is a single number    
    values = [values] if not isinstance(values, list) else values 
    single_item = True if len(values) == 1 else False 
    
    count = len(list_items)
    for i in range(count):
        if single_item:
            values.append(values[0])
        items.append(list_items[i]*values[i]) # <-- bread and butter of this whole operation chief
    return items

def _calculate_waypoint_count(joints, dt = 0.1):
    T = 0
    biggest_joint = 0
    joint_count = len(joints)
    for i in range(joint_count):
        if abs(joints[i]) > biggest_joint:
            biggest_joint = abs(joints[i])
    while T*dt < biggest_joint:
        T+=1
    return T

def check_path(
        start:list,
        stop:list,
        ignore:Optional[list] = None,
        )->tuple:
    """
    tests a movement between a start pose and an end pose.
    if the movement collides with a box, returns false, long with the names of the colliding parts.
    :input start: a set of joints positions to start at
    :input end: a set of joint positions to end at.
    :input ignore: bounding boxes to ignore for collision checks

    :Returns success : True if the path is collision free
    :returns robot_box: if collision, The part of the robot that collides 
    :returns object_box: if collision, The object that is collided with
    """
    if ignore is None: ignore = []
 
    minus_start = [-1*s for s in start]
    joint_deltas = list_sum(stop, minus_start)
    waypoint_count = _calculate_waypoint_count(joint_deltas)

    for dt in range(waypoint_count):
        next_position = list_sum(start, list_multiply(joint_deltas,((dt+1)/waypoint_count)))
        (kaboom, robot_box, object_box) = check_collisions(next_position)
        if kaboom: 
            return (False, robot_box, object_box)
    return (True, None, None)
    

def get_trajectory_any(movement_name: str, pose_type: Literal["basepose", "joints", "offset"]):
    reader = Jsonreader()
    positions = reader.read("recordings")
    waypoints = list()
    i = 0
    try:
        while True:
            waypoints.append(positions[f"{movement_name}_{i}"][pose_type])
            i+=1
    except KeyError:
        if i < 2: # needs at least a start and a stop
            raise RuntimeError(f"{movement_name} is not properly defined")
    return waypoints

def get_trajectory_joints(movement_name: str):
    return get_trajectory_any(movement_name, "joints")

def get_trajectory_matrix(movement_name: str):
    return get_trajectory_any(movement_name, "basepose")


