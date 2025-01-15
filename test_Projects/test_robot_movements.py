import robot_boot_manager
from time import sleep

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():
    # Init robot
    robot_boot_manager.robot_launch(use_real_robot=False)
    
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        accel_time=0.05
    )
    robot_startup()
    # Not required but recommended:
    bot.arm.go_to_home_pose()


    # Put your code here:

    for i in range(10):
        bot.gripper.release()
        sleep(1)
        bot.gripper.grasp()
        sleep(1)
    """"
    Some hints:
    1) use bot.arm.__________ to use most commands.
    (Notable exception: bot.gripper is used to acces the... you know.)
    2) bot.arm.set_ee_cartesian_trajectory(____=____) allows a given movement in x,y,z,pitch,yaw,roll.
    Meaning that the arm moves the given value away from its current position
    3) bot.arm.set_ee_pose_components() allows hardcoding of a position in 3d space
    4) bot.arm.go_to_home_pose() and bot.arm.go_to_sleep_pose() are the only built in movements.
    5) See the other scripts for examples of movement.
    """














    # Close bot, close program:

    # Make sure robot is stable
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    sleep(3) #Delay to give time to gtfo
    # Go home 
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)
    # Shut down bot safely
    robot_shutdown()
    robot_boot_manager.robot_close()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())