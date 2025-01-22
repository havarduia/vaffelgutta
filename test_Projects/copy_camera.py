# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.Wafflebot import Wafflebot
from time import sleep

from kamera.create_matrix_from_apriltag import save_apriltag_matrix

def get_apriltag_pose():
    return [
        [1,0,0,0.3],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ]


def main():
    # Init robot
    bot = Wafflebot(use_real_robot=False)    
    bot.arm.go_to_home_pose()

    # Put your code here:
    running = True
    while running:
        pose = get_apriltag_pose()
        bot.go_to(pose)
        if get_apriltag_pose == False: running = False

    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())