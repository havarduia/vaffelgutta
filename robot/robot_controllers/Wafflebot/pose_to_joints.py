from time import sleep
from functools import partial
from robot.robot_controllers.safety_functions import fix_joint_limits
import numpy as numphy
from typing import Callable


def get_current_joints(bot: "InterbotixManipulatorXS") -> list[list[float]]:
    sleep(0.03) # give some time for the joints to settle 
    bot.arm.capture_joint_positions()
    sleep(0.03) # give some time for the joints to settle 
    current_pose = bot.arm.get_joint_positions()
    return current_pose 

def errorchecking(success: bool, attempt: int, debug: bool) -> bool:
    if success:
        return True
    else:
        if debug:
            print(f"Wafflebot errorchecking: attempt {attempt} failed.")
        return False

def template_try_movement(bot: "InterbotixManipulatorXS", target: list, guess: list = None):    
    joint_targets, success = bot.arm.set_ee_pose_matrix(
        T_sd=target,
        custom_guess= guess,
        execute=False)
    
    joint_targets = fix_joint_limits(joint_targets)
    success = (joint_targets[0] == False)
    return (joint_targets, success)

def make_try_movement(bot: "InterbotixManipulatorXS", target: list):
   return partial(template_try_movement(bot, target))

def double_twister_fix(joints: list, try_movement: Callable):
        """
        For positions that have made over a quarter rotation: 
        Might cause double rotation --> awkward position.
        So retry with forced upright position. Waist ([0]) is exempt this problem.
        """
        temp_joints = joints.copy()
        for i in range(1,6):
            if abs(temp_joints[i]) > numphy.pi/2:
                temp_joints[i] = 0.0        
        return try_movement(temp_joints) 

def shoulder_bender_fix(joints: list, debug_print: bool,try_movement: Callable):
        """
        if the shoulder is bent back and the elbow is pointing up,
         it is likely another unnatural position that should be fixed
        """
        temp_joints = joints.copy()
        if (
            temp_joints[1] < -(numphy.pi/6)   # if shoulder is bent back 
            and not temp_joints[2] > 0.1      # and elbow is not pointing somewhat foreward
        ):   
            if debug_print:
                print(f"Adjusted from state:\n{temp_joints[0]}\n{temp_joints[1]}\n{temp_joints[2]}")
            # turn around waist (joint overflow will be handled down the line)
            temp_joints[0] +=numphy.pi           
            # reset shoulder and elbow
            temp_joints[1] = 1e-6  
            temp_joints[2] = 1e-6
            if debug_print:
                print(f"to:\n{temp_joints[0]}\n{temp_joints[1]}\n{temp_joints[2]}")
        return try_movement(temp_joints)
    

def refine_guess(bot: "InterbotixManipulatorXS",
                target: list[list[str]],
                debug_print:bool
                ) -> tuple[list[str] | None, bool]:        

        try_movement: Callable = make_try_movement(bot, target)
        current_pose = get_current_joints(bot)
        # Try using current position as seed for target joints. Else retry with vanilla guesses
        (temp_joints, success) =  try_movement(guess=current_pose)
        if not success:
            (temp_joints, success) = try_movement(guess=target)
        # error checking
        if errorchecking(success, 1, debug_print):
            target_joints = temp_joints.copy()
        else:
            return (None, False)
        (temp_joints, success) = double_twister_fix(target_joints, try_movement)
        if errorchecking(success, 2, debug_print):
            target_joints = temp_joints.copy()

        (temp_joints, success) = shoulder_bender_fix(target_joints, debug_print, try_movement)
        if errorchecking(success, 3, debug_print):
            target_joints = temp_joints.copy()
        
        (temp_joints, success) = double_twister_fix(target_joints, try_movement)
        if errorchecking(success, 4, debug_print):
            target_joints = temp_joints.copy()
        
        return target_joints, True

# Enable type hinting:
if __name__ == "__main__":
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS