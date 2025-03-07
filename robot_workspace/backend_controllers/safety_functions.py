from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot_workspace.backend_controllers.robot_bounding_boxes import update_robot_bounding_box
from robot_workspace.backend_controllers.file_manipulation import Jsonreader
import numpy as numphy
from os import getcwd

def _test_collision(object1: list, object2: list)-> bool:
    """
    Returns True if two objects intersect, else false.
    """
    x1_start    =   object1[0][0];    x2_start   =  object2[0][0]
    x1_end      =   object1[1][0];    x2_end     =  object2[1][0]
    y1_start    =   object1[0][1];    y2_start   =  object2[0][1]
    y1_end      =   object1[1][1];    y2_end     =  object2[1][1]
    z1_start    =   object1[0][2];    z2_start   =  object2[0][2]
    z1_end      =   object1[1][2];    z2_end     =  object2[1][2]

    if ((x1_start > x2_end) or (x1_end < x2_start)): return False
    if ((y1_start > y2_end) or (y1_end < y2_start)): return False
    if ((z1_start > z2_end) or (z1_end < z2_start)): return False
    #else:
    return True

def read_boxes(name):
    path = getcwd()
    path += f"/robot_workspace/assets/boundingboxes/{name}.py"
    with open(path,"r") as file:
        box_list: dict = eval(file.read(), {"np":numphy})
    boxes = []  
    for key in box_list.keys():
        boxes.append((box_list[key]))
    return (box_list.keys(), boxes)


def _get_joint_limit_map(ind: int, bound_is_upper: bool):
    """
    return limits
    as specified by the documentation at:
    https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300s.html#default-joint-limits
    """
    joint_limit_map_lower =numphy.array([
    -180, #waist
    -101, #shoulder
    -101, #elbow            
    -180, # forearm roll
    -107, # wrist angle
    -180, # wrist rotate
    ])

    joint_limit_map_upper = numphy.array([
    180, #waist
    101, #shoulder
    92, #elbow            
    180, # forearm roll
    130, # wrist angle
    180, # wrist rotate
    ]) 
    
    joint_limit_map_lower = numphy.deg2rad(joint_limit_map_lower)
    joint_limit_map_upper = numphy.deg2rad(joint_limit_map_upper)
    return joint_limit_map_upper[ind] if bound_is_upper else joint_limit_map_lower[ind]

def _adjust_joint_bound(joint: float, ind: int, debug_print: bool = False):
        
    adjusted = False
    #test lower bound
    while joint < -numphy.pi:
        if debug_print:
            print(f"adjusting joint {ind} UP from {joint}")
        joint += numphy.pi*2
        adjusted = True

    #test upper bound
    while joint > numphy.pi:
        if debug_print:
            print(f"adjusting joint {ind} DOWN from {joint}")
        joint -= numphy.pi*2  
        adjusted = True
    
    if adjusted:
        if debug_print:
            print ("final joint state for joint " + str(ind) +": "
            + str(_get_joint_limit_map(ind=ind, bound_is_upper=False))+
              " < " + str(joint) + " < "
                + str(_get_joint_limit_map(ind=ind, bound_is_upper=True))
                +", " + str(_get_joint_limit_map(ind=ind, bound_is_upper=False) < joint < (_get_joint_limit_map(ind=ind, bound_is_upper=True))) 
                )
    
    return joint

def _fix_single_joint(joint: float, ind: int, debug_print:bool = False):    
    lower_bound = _get_joint_limit_map(ind = ind, bound_is_upper=False)
    upper_bound = _get_joint_limit_map(ind = ind,bound_is_upper=True)
    
    joint = _adjust_joint_bound(joint=joint, ind=ind)
    
    # Return error if adjusted joint is out of bounds  
    if (joint < lower_bound
        or joint > upper_bound):
        if debug_print:
            print("Safety_functions: Joint fixer returned an invalid value.")
            print(f"expected: {lower_bound} < joint < {upper_bound}")
            print(f"got: {joint}")
        return False
    if joint == 0: joint = 1e-6 # reserve 0.0 for error messaging
    
    return joint

def fix_joint_limits(joints: list)->list:
    """
    iterates and ensures the joint command is within its given limits
    as specified by the documentation at:
    https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300s.html#default-joint-limits
   
    :input: joints - joint to be limited
    :output: list of joint postions if valid, [False] if invalid 
    """
    ind = 0
    for joint in joints:
        joint = _fix_single_joint(joint=joint, ind=ind)
        if joint == False: return [False] # 0.0 represents error
        joints[ind] = joint
        ind+=1
    
    return joints 

def check_collisions(pose: list, overrides: list = []):
    update_robot_bounding_box(pose)

    reader = Jsonreader("robot_workspace/assets/boundingboxes/")
    robotboxes = reader.read("robot")
    boundingboxes = reader.read("boundingboxes")

    # Test for collision:
    for object_boxname, object_box in zip(boundingboxes.keys(),boundingboxes.values()):
        if object_boxname in overrides: 
            continue
        for robot_boxname, robot_box in zip(robotboxes.keys(), robotboxes.values()):
            if _test_collision(robot_box, object_box): 
                return(True, robot_boxname, object_boxname)
    #if not collision:
    return(False, None, None)
     