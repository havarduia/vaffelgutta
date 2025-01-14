from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from backend_controllers import robot_boot_manager

from time import sleep
def main():
    bot = InterbotixManipulatorXS(robot_model="vx300s",
                                  group_name="arm",
                                  gripper_name="gripper",
                                  accel_time=0.05)
    robot_boot_manager.robot_launch()
    robot_startup()
    sleep(5)
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(5)
    bot.core.robot_torque_enable("group", "arm", True)
    sleep(5)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)
    robot_shutdown()
    robot_boot_manager.robot_close()



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        robot_shutdown()
        try:
            robot_boot_manager.robot_close()
        except NameError:
            print("Error: Program closed without valid PID")
    