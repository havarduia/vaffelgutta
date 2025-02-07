# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets import arm_positions, arm_joint_states
from robot_workspace.backend_controllers import safety_functions as safety 
from time import sleep
import numpy as numphy


def main():
    bot = Wafflebot()
    bot.arm.go_to_home_pose()

    bot.gripper.release()
    bot.big_movement(joint_state_target="prep")
    bot.small_movement("prepare")
    bot.small_movement("grab")
    bot.gripper.grasp()


    bot.big_movement("up")
    bot.arm.set_trajectory_time(accel_time=0.25)
    bot.small_movement("upp")
    bot.small_movement("up")
    bot.small_movement("upp")
    bot.small_movement("up")
    bot.arm.set_trajectory_time(moving_time=2.0)
    sleep(1.5)
    bot.gripper.release()
    sleep(2)
    bot.big_movement("home")
    bot.safe_stop()

    
# Footer:
def handle_error(signum, frame): raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: 
            exec(errorhandler.read())
