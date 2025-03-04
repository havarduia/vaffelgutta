# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath

chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.angle_manipulation import angle_manipulation
from robot_workspace.assets.positions import joint_states, offsets,positions
from robot_workspace.backend_controllers import robot_boot_manager
from robot_workspace.backend_controllers.tf_publisher import publish_tf
from robot_workspace.backend_controllers import safety_functions, path_planner

from rclpy import ok as rclpyok
from time import sleep
import argparse
import threading
import numpy as numphy
from importlib import reload as import_reload

from sys import modules as sysmodules
if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
    import Jetson.GPIO as GPIO


def calculate_translation(startpos: list, endpos: list):
    """Calculate the translation of two 4x4 matrices"""
    x = endpos[0][3] - startpos[0][3]
    y = endpos[1][3] - startpos[1][3]
    z = endpos[2][3] - startpos[2][3]
    return (x, y, z)

def _calculate_biggest_joint(joints):    
    biggest_joint = 0
    for joint in joints:
        joint = abs(joint)
        if joint > biggest_joint:
            biggest_joint = joint
    return biggest_joint


class Wafflebot:
    #Todo: Replace bota.arm.go_to_home_pose og ....sleep_pose with error checked versions. 
    def __init__(self, use_real_robot = False):
        # Include launch arguments 
        parser = argparse.ArgumentParser(description="Runs a wafflebot")
        parser.add_argument("-r", required=False, type=int, default=0, help="1 to use real robot, 0 to simulate")
        args = parser.parse_args()
        self.use_real_robot = bool(args.r)

        if use_real_robot == True: self.use_real_robot = True # Override launch option if program specifies real bot

        robot_boot_manager.robot_launch(use_real_robot=self.use_real_robot)
        self.bot = InterbotixManipulatorXS(
            robot_model= "vx300s",
            group_name="arm",
            gripper_name="gripper",
            )
    
        # Define shorthands to call bot functions intuitively 
        self.arm = self.bot.arm
        self.gripper = self.bot.gripper
        self.core = self.bot.core
        
        # start up robot
        self.arm.capture_joint_positions()
        robot_startup()


        # Monitor emergency stop
        if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
            # **Start GPIO monitoring in a separate thread**
            self.gpio_thread = threading.Thread(
                target=self.monitor_gpio,
                daemon=True
                  )
            self.gpio_thread.start()
    def __del__(self):
        self.exit()
    
    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)

    def _refine_guess(self,target, debug_print: bool = False):
        bot = self.bot
        self.arm.capture_joint_positions()
        current_pose = self.arm.get_joint_positions()
        # Try using current position as seed for target joints. Else retry with vanilla guesses
        (target_joints, success) = path_planner.plan_matrix(bot, target, current_pose)
        if not success:
            target_joints, success = path_planner.plan_matrix(bot, target)
        # error checking
        if not success:
            if debug_print:
                print("Wafflebot: move failed after first fix joints - aborting")
            return (None, False)
        # For positions that have made over a quarter rotation: 
        # Might cause double rotation --> awkward position.
        # So retry with forced upright position. Waist ([0]) is exempt this problem.
        prev_target = target_joints.copy()
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0        
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints) 
        if not success:
            if debug_print:
                print("Wafflebot: move failed after second fix joints - using previous guess")
            target_joints = prev_target
        # if the shoulder is bent back and the elbow is pointing up,
        # it is likely another unnatural position that should be fixed
        prev_target = target_joints.copy()
        if (
            target_joints[1] < -(numphy.pi/6)   # if shoulder is bent back 
            and not target_joints[2] > 0.1      # and elbow is not pointing somewhat foreward
        ):   
            if debug_print:
                print(f"Adjusted from state:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
            # turn around waist (joint overflow will be handled down the line)
            target_joints[0] +=numphy.pi           
            # reset shoulder and elbow
            target_joints[1] = 1e-6  
            target_joints[2] = 1e-6
            if debug_print:
                print(f"to:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
            adjusted = True
        else:
            adjusted = False
        # refine adjusted target pose        
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints) 
        if not success:
            if debug_print:
                print("Wafflebot: move failed after third fix joints - using previous guess")
            target_joints = prev_target
        # since the position has changed, redo the first uprightness test.
        prev_target = target_joints.copy()
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0001
        # refine final target pose
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints)        
        if not success:
            if debug_print:
                print("Wafflebot: move failed after fourth fix joints - using previous guess")
            target_joints = prev_target
        if adjusted:
            if debug_print:
                print(f"the adjusted joints are:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
        return target_joints, True

    
    def _interpret_target_command(self, target):
        import_reload(joint_states)
        import_reload(positions)
        if ( isinstance(target, list) ) and ( len(target) == 6 ):
            return target
        if isinstance(target, numphy.matrix):
            target = target.tolist() 
        if isinstance(target, str):
            target = getattr(positions, target)
        target_joints = self._refine_guess(target)
        return target_joints
    
    def exit(self):
        if rclpyok():
            robot_shutdown()  
            robot_boot_manager.robot_close()
    
    
    def safe_stop(self, slow = False):
        self.arm.set_trajectory_time(moving_time=(8.0 if slow else 2.0)) # reset moving time if changed elsewhere
        self.bot.core.robot_torque_enable("group", "arm", True)
        sleep(2)
        self.arm.go_to_home_pose()
        sleep_joints =  [0.0, -1.80, 1.6, 0.0, 0.5859, 0.0]
        self.arm.set_joint_positions(sleep_joints)
        sleep(0.5)
        self.exit()


    def monitor_gpio(self):
        """ Function to monitor GPIO button in a separate thread. """
        # Set the GPIO mode
        GPIO.setmode(GPIO.BOARD)
        button_pin = 18  # Define button pin
        # Set the pin as an input
        GPIO.setup(button_pin, GPIO.IN)
              
        while True:
            pin_state = GPIO.input(button_pin)
            if pin_state == GPIO.LOW:
                self.safe_stop(slow = True)
                break
            sleep(0.1)  # Prevent CPU overuse
    
    
    def cancel_movement(self):
        current_pose = self.arm.get_ee_pose()
        self.arm.set_ee_pose_matrix(current_pose)


    def move(self,
            target, 
            ignore = [],
            speed_scaling: float = 1.0, 
            blocking: bool = True, 
            debug_print: bool = False
            ) -> None:
        # Todo? add blocking = False?
        start_joints = self.arm.get_joint_positions()
        target_joints, success = self._interpret_target_command(target)

        if not success:
            if debug_print:
                print("Wafflebot: \nEnd pose not found. Cannot plan movement.")
            return False    
        waypoints, success = (path_planner.plan_path(self, start_joints, target_joints, ignore, []))

        if not success: 
            if debug_print:
                print("Wafflebot: move failed after path planner")
            return False
        if not isinstance(waypoints, list):
            return False
        
        if len(waypoints) == 1:
            try: 
                if len(waypoints[0]) == 1:
                    return False
            except TypeError:
                if debug_print:
                    print ("TypeError for testing len(waypoints[0])==1")
        
        speedconstant = 0.42066638
        prev_waypoint = start_joints
        from robot_workspace.backend_controllers.path_planner import _list_sum, _list_multiply
        for waypoint in waypoints:

            joint_travel_distance =_list_sum(waypoint, _list_multiply(prev_waypoint,-1)) 
            wp_path_length = max(_calculate_biggest_joint(joint_travel_distance), 1e-8)
            speed = (speedconstant * speed_scaling / wp_path_length) 
            min_move_time = 0.5
            move_time = max(1.0/speed, min_move_time)
            self.arm.set_joint_positions(waypoint,moving_time=move_time,blocking=False)
            prev_waypoint = waypoint
            if blocking:
                sleep(move_time)













    # Deprecated functions
    def big_movement(self, target: str, target_position_matrix = None): # todo: add support to convert variables to string

        """
        moves the bot to a faraway place. requires a preset waypoint in joint space
        A list of joint states are stored in assets/arm_joint_states.py
        :input: joint_state_target: The joint state to go to
        :input: target_position_matrix: optional parameter to adjust waist position to point towards the given direction  
        """
        print("Warning: big_movement is deprecated. please use move() instead.")
        import_reload(joint_states)
        joint_name = target # save name for future use
        target = self._interpret_target_command(target)
        
        # If a target matrix is given, adjust the joint first  
        if target_position_matrix != None:
            
            # To ensure compatibility - cast to a numphy matrix
            target_position_matrix = numphy.matrix(target_position_matrix)             
            # extract the angle of the position
            target_x = target_position_matrix[3,0]
            target_y = target_position_matrix[3,1]
            target_rad = numphy.atan2(target_y, target_x)
            # commit the position    
            target[0] = target_rad
        
        # Check the feasability of the movement 
        target = safety_functions.fix_joint_limits(target)
        # If an error was raised, abort  """movement
        if target[0] == False: 
            print("big_movement: joint positions not reachable - aborting movement")
        # Else, move.
        else:
            self.arm.set_joint_positions(target)
            self.small_movement(joint_name)
    
    
    def small_movement(self, target): # todo: add support to convert variables to string
        print("warning: small_movement is depreceated. please use move() instead.")
        if isinstance(target, str):
            target = getattr(positions,target)
        if not isinstance (target,list):
            print ("Wafflebot - Small movement: target position is not a valid type")
            return False # error

        self.arm.capture_joint_positions() # in hopes of reminding the bot not to kill itself with its next move

        waypoints = self.arm.set_ee_pose_matrix(target)        


        for waypoint in waypoints:
            joints = self.arm.get_joint_positions()
                        
            joints = self.arm.set_ee_pose_matrix(
                waypoint,
                execute=False,
                custom_guess=joints
                )[0]            
            joints = safety_functions.fix_joint_limits(joints=joints)

  
            if joints[0] == False:
                print("small_movement failed.")
                return
                #todo error handling

            
            # if joint values are "out of wack", retry with upright-er positions 
            for i in range(1,6):
                if abs(joints[i]) > numphy.pi/2:
                    joints[i] = 0.0
            
            joints = self.arm.set_ee_pose_matrix(
                waypoint,
                execute=False,
                custom_guess=joints
                )[0]
            joints = safety_functions.fix_joint_limits(joints=joints)            

        
            if joints[0] == False:
                print("Small_movement failed.")
                return
                #todo error handling
            
            self.arm.set_joint_positions(joints)
        return
  