import robot_boot_manager
from time import sleep

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def main():
    pid = robot_boot_manager.robot_launch()

    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper'
    )
    robot_startup()

    bot.arm.go_to_home_pose()
    ans = bot.arm.set_single_joint_position("shoulder")
    print(ans)
    bot.arm.go_to_sleep_pose()
    robot_boot_manager.close_robot(pid)
    print("finished!")








if __name__=='__main__':
    main()

