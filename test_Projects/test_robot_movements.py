# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from time import sleep
from robot.backend_controllers.robot_controllers.Wafflebot import Wafflebot

def main():
    # Init robot
    bot = Wafflebot()
    # Not required but recommended:
    bot.arm.go_to_home_pose()

    # Put your code here:
    """
    Some hints:
    edit: bot.move() is the new go to movement command - uses a pose matrix.
    1) use bot.arm.__________ to use most commands.
    (Notable exception: bot.gripper is used to acces the... you know.)
    2) bot.arm.set_ee_cartesian_trajectory(____=____) allows a given movement in x,y,z,pitch,yaw,roll.
    Meaning that the arm moves the given value away from its current position
    3) bot.arm.set_ee_pose_components() allows hardcoding of a position in 3d space
    4) bot.arm.go_to_home_pose() and bot.arm.go_to_sleep_pose() are the only built in movements.
    5) See the other scripts for examples of movement.
    """
    from robot.assets.positions import recordings    
    bot.gripper.release()
    bot.gripper.grasp()
    try:
        speed = 1.0
        while True:
            bot.move(recordings.square_a,speed_scaling=speed)
            bot.move(recordings.square_b,speed_scaling=speed)
            bot.move(recordings.square_c,speed_scaling=speed)
            bot.move(recordings.square_d,speed_scaling=speed)
    except KeyboardInterrupt:

    # Close bot, close program:
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
    