from time import sleep
import robot_boot_manager

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():

    rotationmatrix = [
        [0, 1, 0, 0],
        [-1, 0, 0 , 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ] # Rotation about z axis -90 deg
    offsetmatrix = [[0,0, 0,0], 
                    [0,0,0,0],
                    [0,0,0,0.02],
                    [0,0,0,0]
                    ]
    
    robot_boot_manager.robot_launch(use_real_robot=1)
    bot = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        accel_time=0.05,
    )
    try:
        robot_startup()
        bot.arm.go_to_home_pose()
        sleep(0.5)
        n=5
        n = 2*n
        for i in range(0,n):
            currentpos = bot.arm.get_ee_pose_command()
            newpos = (rotationmatrix @ currentpos)
            newpos = newpos+offsetmatrix if (i < n/2) else newpos-offsetmatrix
            bot.arm.set_ee_pose_matrix(newpos)
        sleep(1)
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
        sleep(1.5)
        robot_shutdown()
        robot_boot_manager.robot_close()



    except KeyboardInterrupt:
        robot_boot_manager.robot_close()





if(__name__=='__main__'):
    main()