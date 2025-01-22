# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.angle_manipulation import angle_manipulation

import Jetson.GPIO as GPIO
from robot_workspace.backend_controllers import find_pose_from_matrix, robot_boot_manager
from time import sleep
from robot_workspace.assets import check_safety
import argparse
import threading
from sys import modules as sysmodules


class Wafflebot:
    def __init__(self, use_real_robot = False):
        # Include launch arguments 
        parser = argparse.ArgumentParser(description="Runs a wafflebot")
        parser.add_argument("-r", required=False, type=int, default=0, help="1 to use real robot, 0 to simulate")
        args = parser.parse_args()
        self.use_real_robot = bool(args.r)

        if use_real_robot == True: self.use_real_robot = True # Override launch option if program specifies otherwise

        robot_boot_manager.robot_launch(use_real_robot=self.use_real_robot)
        self.bot = InterbotixManipulatorXS(
            robot_model= "vx300s",
            group_name="arm",
            gripper_name="gripper",
            )
        
        self.arm = self.bot.arm
        self.gripper = self.bot.gripper
        self.core = self.bot.core
        self.arm.capture_joint_positions()
        robot_startup()
        self.arm.capture_joint_positions()
        # Monitor emergency stop
        if "Jetson.GPIO" in sysmodules:
            # **Start GPIO monitoring in a separate thread**
            self.gpio_thread = threading.Thread(
                target=self.monitor_gpio,
                daemon=True
                  )
            self.gpio_thread.start()
    
    
    def __getattr__(self, name):
        return getattr(self.bot, name)
    
    def monitor_gpio(self):
        """ Function to monitor GPIO button in a separate thread. """
        # Set the GPIO mode
        GPIO.setmode(GPIO.BOARD)
        button_pin = 18  # Define button pin
        # Set the pin as an input
        GPIO.setup(button_pin, GPIO.IN)#pull_up_down=GPIO.PUD_DOWN)
        previous_presssed = False                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        while True:
            pin_state = GPIO.input(button_pin)
            if not previous_presssed: # edge detction
                if pin_state == GPIO.LOW:
                    from os import kill, getpid
                    from signal import SIGINT
                    kill(getpid(), SIGINT)
            previous_presssed = True if pin_state == GPIO.LOW else False
            sleep(0.1)  # Prevent CPU overuse
    
    
    def exit(self):
        robot_shutdown()  
        robot_boot_manager.robot_close()
    

    def cancel_movement(self):
        current_pose = self.arm.get_ee_pose()
        self.arm.set_ee_pose_matrix(current_pose)
    
    
    def safe_stop(self):
        self.bot.core.robot_torque_enable("group", "arm", True)
        sleep(2)
        self.arm.go_to_home_pose()
        self.arm.go_to_sleep_pose()
        sleep(0.5)
        self.exit()

    def go_to(self, target):
        self.arm.capture_joint_positions() # in hopes of reminding the bot not to kill itself with its next move
        
        target_matrix = find_pose_from_matrix.compute_relative_pose(target, self.arm.get_ee_pose())
        target_matrix = check_safety.check_safety(target_matrix)
        if target_matrix[3][0] == 1: return False # if safety bit is 1, cancel movement 
        goal_pose = find_pose_from_matrix.find_pose_from_matrix(target_matrix)
        
        if not (self.arm.set_ee_cartesian_trajectory(
            x = goal_pose.x,
            y = goal_pose.y,
            z = goal_pose.z,
            roll = goal_pose.roll,
            pitch = goal_pose.pitch,
            yaw = goal_pose.yaw,
            )):
            self.arm.set_ee_pose_matrix(target)
        self.arm.capture_joint_positions() # in hopes of reminding the bot not to kill itself with its next move
        return