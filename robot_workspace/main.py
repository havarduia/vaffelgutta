# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets import arm_positions
from time import sleep

def main():
    
    bot = Wafflebot()
    
    bot.arm.go_to_home_pose()
    bot.gripper.release()
    
    bot.go_to(arm_positions.pgrab())
    bot.go_to(arm_positions.grab())
    bot.gripper.grasp()
    bot.go_to(arm_positions.postgrab())
    bot.arm.go_to_home_pose()
    bot.go_to(arm_positions.spray())
    sleep(1)
    bot.go_to(arm_positions.sprayy())
    sleep(1)
    bot.arm.go_to_home_pose()
    bot.go_to(arm_positions.pgrab())
    bot.go_to(arm_positions.grab())
    bot.gripper.release()
    bot.go_to(arm_positions.postgrab())


    bot.safe_stop()



if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())