# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from time import sleep

def main(bot,cam,aruco,coordsys):
    # Not required but recommended:
    bot.go_to_home_pose()

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




# footer:

    bot.safe_stop()


if __name__ == '__main__':
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from camera.init_camera import initalize_system as init_camera
    from rclpy.exceptions import InvalidHandle
    from robot.tools.errorhandling import handle_error
    try:
        cam, aruco, coordsys = init_camera()
        bot = Wafflebot(coordsys)
        main(bot=bot,cam=cam,aruco=aruco,coordsys=coordsys)
    # if error detected, run the error handler
    except (InvalidHandle):
        pass
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)
