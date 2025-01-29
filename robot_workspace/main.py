# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets import arm_positions
from robot_workspace.backend_controllers import safety_functions as safety 
from time import sleep
import numpy as numphy
from camera.camera import ArucoPoseLauncher

def main():
    camera = ArucoPoseLauncher("/home/havard/git/vaffelgutta/camera/run_camera.sh")
    bot = Wafflebot()
    camera.launch()
    bot.arm.go_to_home_pose()
    sleep(120)
    bot.safe_stop()
    camera.kill()
    
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
