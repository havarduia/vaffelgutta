# Change the working directory to the base directory
from os import chdir, getcwd
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# interbotix modules
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# user libraries: 
from robot.backend_controllers import robot_boot_manager
from time import sleep
from robot.assets.Wafflebot import Wafflebot
def main():
    bot = Wafflebot()
    
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(2)
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(5)
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    savedpose = bot.arm.get_ee_pose() 
    print(savedpose)
    sleep(5)
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_matrix(savedpose)
    bot.arm.go_to_home_pose()

    bot.safe_stop()



# Footer:
def handle_error(signum, frame):raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
    
