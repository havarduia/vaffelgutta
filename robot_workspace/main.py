# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets import arm_positions
from time import sleep
import numpy as numphy

def main():
    
    bot = Wafflebot()
    
    bot.arm.go_to_home_pose()
    from robot_workspace.backend_controllers.tf_publisher import publish_tf
    from robot_workspace.assets import camera_readings
    from importlib import reload as import_reload

    while True:
        import_reload(camera_readings)
        publish_tf(camera_readings.test_marker)
        bot.arm.set_ee_pose_matrix((camera_readings.test_marker), blocking=False)
    
    from robot_workspace.backend_controllers import safety_functions as safety
    lim = bot.arm.get_joint_positions()
    safety.fix_joint_limits(lim)
    
    bot.safe_stop()
    
# Footer:
def handle_error(signum, frame):raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
    
