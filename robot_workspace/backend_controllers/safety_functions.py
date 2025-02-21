from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy
from importlib import reload as import_reload
from robot_workspace.assets.boundingboxes import robot as robotboxes
from robot_workspace.assets.boundingboxes import boundingboxes
from robot_workspace.backend_controllers.robot_bounding_boxes import update_robot_bounding_box
from os import getcwd
import inspect

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

    if ((x1_start > x2_end) and (x1_end < x2_start)):
        if ((y1_start > y2_end) and (y1_end < y2_start)):
            if ((z1_start > z2_end) and (z1_end < z2_start)):
                return False
    return True

def read_boxes(name):
    path = getcwd()
    path += f"/robot_workspace/assets/boundingboxes/{name}.py"
    with open(path,"r") as file:
        box_list: dict = eval(file.read(), {"np":numphy})
    boxes = []  
    for key in box_list.keys():
        boxes.append((box_list[key]))
    return boxes


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

    joint_limit_map_upper =numphy.array([
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

def _adjust_joint_bound(joint, ind: int):
    
    lower_bound = _get_joint_limit_map(ind = ind, bound_is_upper=False)
    upper_bound = _get_joint_limit_map(ind = ind,bound_is_upper=True)
    
    adjusted = False
    #test lower bound
    while joint < lower_bound:
        print(f"l. bound = {lower_bound}")
        print(f"curr. joint = {joint}")
        print(f"adjusting joint {ind} UP")
        joint += numphy.pi*2
        adjusted = True

    #test upper bound
    while joint > upper_bound:
        print(f"curr. joint = {joint}")
        print(f"u. bound = {upper_bound}")
        print(f"adjusting joint {ind} DOWN")
        joint -= numphy.pi*2  
        adjusted = True
    
    if adjusted:
        print ("final joint state for joint " + str(ind) +": "
            + str(_get_joint_limit_map(ind=ind, bound_is_upper=False))+
              " < " + str(joint) + " < "
                + str(_get_joint_limit_map(ind=ind, bound_is_upper=True))
                +", " + str(_get_joint_limit_map(ind=ind, bound_is_upper=False) < joint < (_get_joint_limit_map(ind=ind, bound_is_upper=True))) 
                )
    
    return joint



def _fix_single_joint(joint: float, ind: int):    
    lower_bound = _get_joint_limit_map(ind = ind, bound_is_upper=False)
    upper_bound = _get_joint_limit_map(ind = ind,bound_is_upper=True)
    
    joint = _adjust_joint_bound(joint=joint, ind=ind)     
    
    # Return error if adjusted joint is out of bounds  
    if (joint < lower_bound
        or joint > upper_bound):
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

def _valid_box_names_test(boxnames, banned_names = []):
    names = []
    for name, obj in boxnames:
        skip = False
        if name.startswith("__"):   skip = True
        if inspect.isfunction(obj): skip = True 
        if inspect.isclass(obj):    skip = True
        for banned_name in banned_names:
            if name == banned_name: skip = True; break # dont continue checking if a banned name has been found 
        
        if not skip:
            names.append(name)

    return names
        

def check_collisions(bot: InterbotixManipulatorXS, pose: list, overrides: list = []):
    update_robot_bounding_box(pose)
    
    import_reload(robotboxes)
    import_reload(boundingboxes)
    robot_boxes = read_boxes("robot")
    boxnames = inspect.getmembers(boundingboxes)
    
    valid_boxnames = _valid_box_names_test(boxnames)
#   valid_boxnames = ([name for name, obj in boxnames if not inspect.isfunction(obj) and not inspect.isclass(obj) and not name.startswith("__")])
    collisionobjects = []
    for name in valid_boxnames:
        collisionobjects.append(getattr(boundingboxes,name))

    # Test for collision:
    for robot_box in robot_boxes:
        for object in collisionobjects:
            if _test_collision(robot_box, object): 
                return(True, robot_box, object)
    #else:
    return(False, None, None)
     