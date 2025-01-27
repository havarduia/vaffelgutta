from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy

def check_collisions(bot: InterbotixManipulatorXS, start_pose_matrix, end_pose_matrix, overrides: list[str] = "None"):
    waypoints = list()
    target_joint_positions = bot.arm.set_ee_pose_matrix(end_pose_matrix, execute=False)[0]
    
    start_position_vector = start_pose_matrix[3,:3] 
    end_position_vector = end_pose_matrix[3,:3] 
    delta_position_vector = end_position_vector-start_position_vector
    # Test for collisions
    N_max = 20
    for N in range(0,N_max):
        step =  N*delta_position_vector/N_max  
        LERP = start_position_vector + step
        # todo: check if it hit anything
    
    # extract pose parameters

    
    # test for floor collision
    while end_pose_matrix[2][3] <= 0.03: # If z is below floor height
        # Reset to a taller place
        end_pose_matrix[2][3] += 0.001 

    waypoints.append(end_pose_matrix)
    return waypoints

def _fix_single_joint(joint: float, ind: float):
    """
    iterates and ensures the joint command is within its given limits
    as specified by the documentation at:
    https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300s.html#default-joint-limits
   
    :input: joint - joint to be limited
    :input: ind - the index of the joint
    """
    joint_limit_map_lower =numphy.array[
    -180, #waist
    -101, #shoulder
    -101, #elbow            
    -107, # wrist angle
    -180, # forearm roll
    -180, # wrist rotate
    ] 
    joint_limit_map_upper =numphy.array[
    180, #waist
    101, #shoulder
    92, #elbow            
    130, # wrist angle
    180, # forearm roll
    180, # wrist rotate
    ] 

    joint_limit_map_lower = numphy.deg2rad(joint_limit_map_lower)
    joint_limit_map_upper = numphy.deg2rad(joint_limit_map_upper)

    lower_fixed = False
    upper_fixed = False

    #test lower bound
    while not lower_fixed:
        if joint < joint_limit_map_lower[ind]:
            joint += numphy.pi*2
        else: lower_fixed = True
    #test upper bound
    while not upper_fixed:
        if joint > joint_limit_map_upper[ind]:
            joint -= numphy.pi*2
        else: upper_fixed_fixed = True
    if upper_fixed and lower_fixed:
        if (joint < joint_limit_map_lower
            and joint > joint_limit_map_upper):
            return False
        elif joint == 0: joint += 1e-6
    else:
        return joint

def fix_joint_limits(joints: list)->list:
    return 0