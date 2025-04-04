from time import sleep
from functools import partial
from robot.robot_controllers.safety_functions import fix_joint_limits
import numpy as numphy
from typing import Callable
from modern_robotics import IKinSpace
from multiprocessing import Process, Manager, Queue
from robot.tools.timekeeper import read_times, record_time
from copy import deepcopy
from random import random
from time import time_ns

def get_current_joints(bot: "InterbotixManipulatorXS") -> list[list[float]]:
    sleep(0.03) # give some time for the joints to settle 
    bot.arm.capture_joint_positions()
    current_pose = bot.arm.get_joint_positions()
    return current_pose 

def errorchecking(success: bool, attempt: int, debug: bool) -> bool:
    if success:
        return True
    else:
        if debug:
            print(f"Wafflebot errorchecking: attempt {attempt} failed.")
        return False

def movement_proc_caller(q: Queue, bot: "InterbotixManipulatorXS", target_pose: list, guess: list = None):
        result = IKinSpace(
            bot.arm.robot_des.Slist,
            bot.arm.robot_des.M,
            target_pose,
            guess,
            0.001,
            0.001
            )
        q.put(result)
        return
        

def template_try_movement(bot: "InterbotixManipulatorXS", target_pose: list, guess: list = None):    
    guesses = deepcopy(bot.arm.initial_guesses)
    if guess is not None:
        guesses.append(guess)
    results = []
    q = Queue(len(guesses))
    for i in range(len(guesses)):
        try:
            proc = Process(target=movement_proc_caller,args=(q,bot,target_pose,guess))    
            proc.start()
            results.append(q.get(timeout=1))
        except TimeoutError:
             proc.close()
    proc.join()
    q.close()

    for joints, success in results:#[r for r in results]:
        if success:
            joint_targets = fix_joint_limits(joints)
            if joint_targets is not False:
                return (joint_targets, True)
    return (None, False)

def make_try_movement(bot: "InterbotixManipulatorXS", target: list):
   return partial(template_try_movement, bot, target)

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
                target: list[list[float]],
                debug_print:bool
                ) -> tuple[list[str] | None, bool]:        
        record_time("start")

        try_movement: Callable = make_try_movement(bot, target)
        current_pose = get_current_joints(bot)

        # Try using current position as seed for target joints. Else retry with vanilla guesses
        (temp_joints, success) =  try_movement(guess=current_pose)
        record_time(f"move1")
        
        # error checking
        if errorchecking(success, 1, debug_print):
            target_joints = temp_joints.copy()
        else:
            record_time("error")
            read_times()
            return (None, False)
        """
        (temp_joints, success) = double_twister_fix(target_joints, try_movement)
        if errorchecking(success, 2, debug_print):
            target_joints = temp_joints.copy()
        """
        (temp_joints, success) = shoulder_bender_fix(target_joints, debug_print, try_movement)
        if errorchecking(success, 3, debug_print):
            target_joints = temp_joints.copy()
            record_time(f"move2")
        
        (temp_joints, success) = double_twister_fix(target_joints, try_movement)
        if errorchecking(success, 4, debug_print):
            target_joints = temp_joints.copy()
            record_time(f"move3")
        record_time("finish")
        read_times()        
        return target_joints, True

# Enable type hinting:
if __name__ == "__main__":
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS