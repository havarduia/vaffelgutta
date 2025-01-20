# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

import numpy as np
#interbotix libraries
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# user libraries:
from robot_workspace.backend_controllers import robot_boot_manager
from time import sleep

from kamera.create_matrix_from_apriltag import save_apriltag_matrix

import numpy as np
def main():
    # Init robot
    robot_boot_manager.robot_launch(use_real_robot=False)
    
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        accel_time=0.05
    )
    robot_startup()
    # Not required but recommended:
    bot.arm.go_to_home_pose()

    # Put your code here:
    """
    try:
        test = bot.arm.set_ee_pose_matrix([
            [1, 0, 0, 0],
            [1, 1, 0, 1],
            [1, 0, 1, 0.5],
            [0,0,0, 1],
        ])
    except:
        pass
    """
    # Close bot, close program:

    # Make sure robot is stable
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    sleep(3) #Delay to give time to gtfo
    # Go home 
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)
    # Shut down bot safely
    robot_shutdown()
    robot_boot_manager.robot_close()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())