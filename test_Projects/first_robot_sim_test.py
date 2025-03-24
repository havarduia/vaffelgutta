'''
Tests the robot movement through some random ee_pose_trajectory commands
starts rviz automatically,
but must be closed manually with ctrl+c
'''

# To open the terminal:
import robot_boot_manager # to open/close rviz

# interbotix libraries:
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# other libraries:
from time import sleep


def main():
    use_real_robot = False
    robot_boot_manager.robot_launch(use_real_robot=use_real_robot)
    bot = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        accel_time=0.05
    )
    try:  
        robot_startup()
        bot.arm.go_to_home_pose()
        bot.arm.set_single_joint_position("shoulder", -0.3)
        sleep(2)
        return
        bot.arm.set_ee_cartesian_trajectory(x = 0.1)
        sleep(0.5)
        bot.arm.set_ee_cartesian_trajectory(y = 0.1)
        bot.arm.set_ee_cartesian_trajectory(x = -0.1)
        sleep(0.1)
        bot.arm.set_ee_cartesian_trajectory(x = 0.1)
        sleep(0.1)
        bot.arm.go_to_sleep_pose()
        sleep(0.5)
        robot_shutdown()
        robot_boot_manager.robot_close()
        print("shutdown complete")
    except KeyboardInterrupt:
        robot_boot_manager.robot_close() 
    
















# python standard for some reason
if (__name__=='__main__'):
    main()

