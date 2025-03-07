from robot_workspace.backend_controllers.safety_functions import check_collisions, fix_joint_limits
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy

def calculate_biggest_joint(joints):    
    biggest_joint = 0
    for joint in joints:
        joint = abs(joint)
        if joint > biggest_joint:
            biggest_joint = joint
    return biggest_joint


def plan_matrix(bot: InterbotixManipulatorXS, target: list, guess: list = None):
    
    joint_targets, success = bot.arm.set_ee_pose_matrix(
        T_sd=target,
        custom_guess= guess,
        execute=False
    )
    joint_targets = fix_joint_limits(joint_targets)
    if joint_targets[0] == False:
        success = False

    return (joint_targets, success)


def _eliminate_obvious_false_positions(bot, start, stop, ignore: list):    
    """
    ignore.append("floor")
    
    boom, botbox, itembox = check_collisions(bot, start,ignore)  
    if boom: return f"invalid origin - {botbox} collides with {itembox}"
    
    boom, botbox, itembox = check_collisions(bot, stop, ignore)
    if boom: return f"invalid endpoint - {botbox} collides with {itembox}"
    """
    return None

def _list_sum(list_items, values):
    items = []
    # ensure list compatibility and check if the value is a single number    
    values = [values] if not isinstance(values, list) else values 
    single_item = True if len(values) ==1 else False 
 
    count = len(list_items)
    for i in range(count):
        if single_item:
            values.append(values[0])
        items.append(list_items[i]+values[i]) # <-- bread and butter of this whole operation chief
    return items

def _list_multiply(list_items, values):
    items = []
    # ensure list compatibility and check if the value is a single number    
    values = [values] if not isinstance(values, list) else values 
    single_item = True if len(values) ==1 else False 
    
    count = len(list_items)
    for i in range(count):
        if single_item:
            values.append(values[0])
        items.append(list_items[i]*values[i]) # <-- bread and butter of this whole operation chief
    return items

def _calculate_waypoint_count(joints, dt = 1e-1):
    T = 0
    biggest_joint = 0
    joint_count = len(joints)
    for i in range(joint_count):
        if abs(joints[i]) > biggest_joint:
            biggest_joint = abs(joints[i])
    while T*dt < biggest_joint:
        T+=1
    return T

def plan_path(
        bot: InterbotixManipulatorXS,
        start:list,
        stop:list,
        ignore:list = [],
        waypoints:list = [],
        failed_attempts:int = 0,
        timeout: int = 0,
        debug_print: bool = False
        )->list:
    """
    plans a movement between a start pose and an end pose.
    may recursively call itself to append waypoints to a partially planned path.    
    
    :input start: a set of joints positions to start at
    :input end: a set of joint positions to end at.
    :input ignore: bounding boxes to ignore for collision checks
    :input waypoints: List of previously computed waypoints. used for recursive calls
    :input attempt: a way to 
    :Returns: 
    list of waypoint positions if success,
    [False] if failure 
    """
    if timeout > 100:
        if debug_print:
            print("Path planner: timed out")
        return (None, False)
    if not waypoints == []: #Dont test obvious positions twice
        error = _eliminate_obvious_false_positions(bot, start, stop, ignore)
        if not error == None:
            if debug_print:
                print("Path planner: This path is impossible")
                print(f"Reason: {error}")
            return (None, False)    
 
    minus_start = [-1*s for s in start]
    joint_deltas = _list_sum(stop, minus_start)
    waypoint_count = _calculate_waypoint_count(joint_deltas)
    if waypoint_count == 0:
        return ([start], True)
    kaboom = 0
    for dt in range(waypoint_count):
        
        #current_position = _list_sum(start, _list_multiply(joint_deltas, dt/waypoint_count))
        next_position = _list_sum(start, _list_multiply(joint_deltas,((dt+1)/waypoint_count)))
        
        (kaboom, robot_box, object_box) = check_collisions(next_position)

        if kaboom: 
            break
    

    if kaboom:
        if debug_print:
            print("Kaboom!")
            print(f"{robot_box} collided with {object_box}")
        
        if failed_attempts == 0:       
            
            uprighter_position =[0,-0.2,-0.2,0,0,0]

            position_attempt = _list_sum(start,uprighter_position)

            plan = plan_path(bot, start, position_attempt, ignore, waypoints,1, timeout+1)
            if plan[1] == False:
                return[None, False]
            else: 
                waypoints.append(plan[0][len(plan[0])-1])

        elif failed_attempts == 1:
            # preset values recorded as the arm folded in on itself
            position_attempt =[-0.0782330259680748, -0.21168935298919678, 1.3897866010665894, -0.08283496648073196, -1.6106798648834229, 0.0] # waist irrelevant 
            waist_index = 5
            waist_angle = start[waist_index] 
            position_attempt[waist_index] = waist_angle
            plan = plan_path(bot, start,position_attempt,ignore, waypoints, 2, timeout+1)
            if plan[1] == False:
                return [None,False]
            else: 
                waypoints.append(plan[0][len(plan[0])-1])
        elif failed_attempts == 2:
            # preset values recorded as the arm rising up to the clouds bro
            position_attempt = [-0.08130098134279251, -0.2991262674331665, -1.036971092224121, 0.13499031960964203, 1.372912883758545, 0.0]# waist irrelevant
            waist_index = 5
            waist_angle = start[waist_index]
            position_attempt[waist_index] = waist_angle
            plan = plan_path(bot,start,position_attempt,ignore, waypoints, 3, timeout+1)
            if plan[1] == False:
                return [None, False]
            else:
                waypoints.append(plan[0][len(plan[0])-1])

        else:
            #at this point you are beyond saving
            if debug_print:
                print("Movement planner failed - attempts exhausted")
            return None, False
            
        if waypoints[1] == True:
            # continue path planning from the new start point
            plan_path(bot, waypoints[len(waypoints)-1],stop,ignore,waypoints,0, timeout+1)
    else:
        waypoints.append(next_position)   
  
    return waypoints, True
