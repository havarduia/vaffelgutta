# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath

chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.angle_manipulation import angle_manipulation
from robot_workspace.assets.positions import arm_joint_states, arm_offsets

from robot_workspace.assets.positions import arm_positions
from robot_workspace.backend_controllers import robot_boot_manager
from robot_workspace.backend_controllers.tf_publisher import publish_tf
from robot_workspace.backend_controllers import safety_functions, path_planner

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
        self.arm.capture_joint_positions()
        
        # Monitor emergency stop
        if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
            # **Start GPIO monitoring in a separate thread**
            self.gpio_thread = threading.Thread(
                target=self.monitor_gpio,
                daemon=True
                  )
            self.gpio_thread.start()
    
    # return the methods of the child class
    def __getattr__(self, name):
        return getattr(self.bot, name)
    
    def _interpret_target_command(self, target):
        import_reload(arm_joint_states)
        import_reload(arm_positions)
        if ( isinstance(target, list) ) and ( len(target) == 6 ):
                return target
        if isinstance(target, str):
            target = getattr(arm_positions, target)

        # get a pose estimate
        (target_joints, success) = self.arm.set_ee_pose_matrix(
            target,
            custom_guess=self.arm.get_joint_positions(),
            execute=False,
            )

        if not success:
            target_joints = self.arm.set_ee_pose_matrix(
                target,
                execute=False,
                )[0]   
        # fix joints to legal states           
        target_joints = safety_functions.fix_joint_limits(joints=target_joints)

        # error checking
        if target_joints[0] == False:
            print("Wafflebot: move failed after first fix joints")
            return False
                    
        # if joint values are "out of wack", retry with upright-er positions 
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0
        print(target_joints[1])
        if target_joints[1] < -(numphy.pi/3):
            target_joints[1] =0.0
            target_joints[0] +=numphy.pi

        # refine final target pose        
        target_joints = self.arm.set_ee_pose_matrix(
            target,
            execute=False,
            custom_guess=target_joints
            )[0]
        target_joints = safety_functions.fix_joint_limits(joints=target_joints)            

        if target_joints[0] == False:
            print("Wafflebot: move failed after second fix joints")
            return False
        
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0
        # refine final target pose        
        target_joints = self.arm.set_ee_pose_matrix(
            target,
            execute=False,
            custom_guess=target_joints
            )[0]
        target_joints = safety_functions.fix_joint_limits(joints=target_joints)            

        if target_joints[0] == False:
            print("Wafflebot: move failed after third fix joints")
            return False

        return target_joints

    
    def exit(self):
        robot_shutdown()  
        robot_boot_manager.robot_close()
    
    
    def safe_stop(self, slow = False):
        self.arm.set_trajectory_time(moving_time=(8.0 if slow else 2.0)) # reset moving time if changed elsewhere
        self.bot.core.robot_torque_enable("group", "arm", True)
        sleep(2)
        self.arm.go_to_home_pose()
        self.arm.go_to_sleep_pose()
        sleep(0.5)
        self.exit()


    def monitor_gpio(self):
        """ Function to monitor GPIO button in a separate thread. """
        # Set the GPIO mode
        GPIO.setmode(GPIO.BOARD)
        button_pin = 18  # Define button pin
        # Set the pin as an input
        GPIO.setup(button_pin, GPIO.IN)#punll_up_down=GPIO.PUD_DOWN)
              
        while True:
            pin_state = GPIO.input(button_pin)
            if pin_state == GPIO.LOW:
                self.safe_stop(slow = True)
                break
            sleep(0.1)  # Prevent CPU overuse
    
    
    def cancel_movement(self):
        current_pose = self.arm.get_ee_pose()
        self.arm.set_ee_pose_matrix(current_pose)


    def move(self, target, ignore = []):
        # Todo? add blocking = False?
        
        start_joints = self.arm.get_joint_positions()

        target_joints = self._interpret_target_command(target)
        print(target_joints)

        waypoints = path_planner.plan_path(self, start_joints, target_joints, ignore, [])
    
        if not waypoints:
            print("Wafflebot: move failed after path planner")
            return
        end_ind = len(waypoints)-1
        end_of_trajectory = waypoints[end_ind]

        self.arm.set_joint_positions(end_of_trajectory)














    # Deprecated functions
    def big_movement(self, target: str, target_position_matrix = None): # todo: add support to convert variables to string

        """
        moves the bot to a faraway place. requires a preset waypoint in joint space
        A list of joint states are stored in assets/arm_joint_states.py
        :input: joint_state_target: The joint state to go to
        :input: target_position_matrix: optional parameter to adjust waist position to point towards the given direction  
        """
        import_reload(arm_joint_states)
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
        if isinstance(target, str):
            target = getattr(arm_positions,target)
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
  