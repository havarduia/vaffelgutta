from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy

def check_collisions(bot: InterbotixManipulatorXS, start_pose_matrix, end_pose_matrix, overrides: list[str] = "None"):
    waypoints = list()
    
    start_pose_matrix = numphy.matrix(start_pose_matrix)
    end_pose_matrix = numphy.matrix(end_pose_matrix)

    start_position_vector = start_pose_matrix[3,:3] 
    end_position_vector = end_pose_matrix[3,:3] 
    delta_position_vector = end_position_vector-start_position_vector
    # Test for collisions
    N_max = 20
    for N in range(0,N_max):
        step =  N*delta_position_vector/N_max  
        LERP = start_position_vector + step
        # todo: check if it hit anything
    
    # test for floor collision
    while (end_pose_matrix[2,3] <= 0.03): # If z is below floor height
        # Reset to a taller place
        end_pose_matrix[2,3] += 0.001 

    waypoints.append(end_pose_matrix.tolist())
    return waypoints


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
