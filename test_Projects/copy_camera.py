# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
from threading import Thread
from robot_workspace.assets.Wafflebot import Wafflebot
from time import sleep
from robot_workspace.assets.positions import camera_readings
from importlib import reload as import_reload

def get_aruco_pose():
    try:
        import_reload(camera_readings)
        return getattr(camera_readings, "tag_25")
    except AttributeError:
        print("tag 25 not found")
        return False
    
def main():
    # Init robot
    bot = Wafflebot(use_real_robot=False)    
    bot.arm.go_to_home_pose()
    # Put your code here:
    running = True
    i = 1
    while running:
        i+=1
        pose = get_aruco_pose()
        bot.move(pose)
        if i == 2000: running = False
        sleep(0.1)
    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())