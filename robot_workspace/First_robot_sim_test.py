'''
Tests the robot movement through some random ee_pose_trajectory commands
starts rviz automatically,
but must be closed manually with ctrl+c
'''


# To open the terminal:
from subprocess import Popen

# interbotix libraries:
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# other libraries:
from time import sleep


def main():
    
    #run shell script
    Popen(["gnome-terminal", "--", "sh", "-c", 
        'ros2 launch interbotix_xsarm_control'
        +' xsarm_control.launch.py'
        +' robot_model:=vx300s'
        +' hardware_type:=fake'
        +' use_sim:=true'
        #+' &' # allows interactive terminal
          +"; bash"])
    
    bot = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
    )
    robot_startup()
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_cartesian_trajectory(x = 0.1)
    sleep(0.5)
    bot.arm.set_ee_cartesian_trajectory(y = 0.1)
    bot.arm.set_ee_cartesian_trajectory(x = -0.1)
    sleep(0.1)
    bot.arm.set_ee_cartesian_trajectory(x = -0.1)
    sleep(0.1)
    bot.arm.go_to_sleep_pose()
    robot_shutdown()
    print("shutdown complete")
    
    
















# python standard for some reason
if (__name__=='__main__'):
    main()

