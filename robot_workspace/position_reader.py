from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from backend_controllers import robot_boot_manager

from time import sleep
def main():
    robot_boot_manager.robot_launch(use_real_robot=False)
    bot = InterbotixManipulatorXS(robot_model="vx300s",
                                  group_name="arm",
                                  gripper_name="gripper",
                                  accel_time=0.05)
    
    robot_startup()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(2)
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(5)
    bot.core.robot_torque_enable("group", "arm", True)
    print(bot.arm.get_ee_pose())
    sleep(5)
    
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)

    robot_shutdown()
    robot_boot_manager.robot_close()



if __name__ == '__main__':
    # Change the working directory to the base directory
    from os import chdir, path; chdir(path.expanduser("~/git/vaffelgutta"))
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling") as errorhandler: exec(errorhandler.read())