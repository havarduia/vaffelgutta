# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from time import sleep

def main(bot):
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

    from robot.robot_controllers.path_planner import get_trajectory_joints, get_trajectory_matrix
    bot.release()
    bot.move("front_of_waffle_iron")
    sleep(1)
    bot.move("open_waffle_iron_0")
    sleep(1)
    bot.grasp()
    trajectory = get_trajectory_joints("open_waffle_iron")
    trajectory = get_trajectory_matrix("open_waffle_iron")
    sleep(1)
    for waypoint in trajectory:
        bot.move(waypoint,speed_scaling = 2.0/0.02) 
    trajectory.reverse()
    for waypoint in trajectory:
        bot.move(waypoint, speed_scaling =2.0/0.02)
    bot.move(trajectory[-1], speed_scaling = 1)
    bot.release()
    sleep(1)
    bot.move("front_of_waffle_iron")



    # footer:

    bot.safe_stop()


if __name__ == '__main__':
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from rclpy.exceptions import InvalidHandle
    from robot.tools.errorhandling import handle_error
    try:
        bot = Wafflebot(automatic_mode=True, detect_collisions=False)
        main(bot=bot)
    # if error detected, run the error handler
    except (InvalidHandle):
        pass
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)
