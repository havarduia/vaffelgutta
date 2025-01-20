# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.angle_manipulation import angle_manipulation

from robot_workspace.backend_controllers import find_pose_from_matrix, robot_boot_manager
from time import sleep
from robot_workspace.assets import check_safety

class Wafflebot:
    def __init__(self, use_real_robot = False):
        robot_boot_manager.robot_launch(use_real_robot=use_real_robot)
        self.bot = InterbotixManipulatorXS(
            robot_model= "vx300s",
            group_name="arm",
            gripper_name="gripper",
            accel_time=0.05 # to allow weak PSU
            )
        self.arm = self.bot.arm
        self.gripper = self.bot.gripper
        robot_startup()

    def __getattr__(self, name):
        return getattr(self.bot, name)
    
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
        
        goal_pose = find_pose_from_matrix.find_pose_from_matrix(target_matrix)
        
        self.arm.set_ee_cartesian_trajectory(
            x = goal_pose.x,
            y = goal_pose.y,
            z = goal_pose.z,
            roll = goal_pose.roll,
            pitch = goal_pose.pitch,
            yaw = goal_pose.yaw,
            )
        self.arm.capture_joint_positions() # in hopes of reminding the bot not to kill itself with its next move
        return