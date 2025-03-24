import rclpy.executors
from test_Projects.moveit_test.new_moveit_test import MotionPlanner
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot.robot_controllers.robot_boot_manager import robot_launch, robot_close
from robot.tools.errorhandling import handle_error
import rclpy
from threading import Thread
from time import sleep
from robot.tools.file_manipulation import Jsonreader
import numpy as numphy

def main():
    homepose = numphy.identity(4)
    homepose[0,3] = 0.4
    homepose[1,3] = 0.0
    homepose[2,3] = 0.45
    homepose = homepose.tolist()
    
    rclpy.init()

    interbotix_moveit_process = robot_launch(use_real_robot=1)
    sleep(5)
    bot = InterbotixManipulatorXS(robot_model="vx300s",)

    sleep(2)

    motionplanner = MotionPlanner(interbotix_moveit_process)
    motionplanner.move(0,1)
    return
    reader = Jsonreader()
    poses = reader.read("recordings")
    #motionplanner.move(bot.arm.get_joint_positions(), homepose)
    sleep(2)
    bot.arm.capture_joint_positions()
    print(bot.arm.get_joint_positions())
    print(bot.arm.get_joint_commands())
    

   # motionplanner.move(bot.arm.get_joint_commands(),poses["hagle"]["matrix"])
    sleep(30)
       
    motionplanner.destroy_node()
    robot_close()
    return
    bot.arm.capture_joint_positions()
    print(bot.arm.get_joint_commands())
    print(bot.arm.get_joint_positions())

    motionplanner.move(bot.arm.get_joint_commands(),poses["zestyheil"]["matrix"])
    #sleep(3)
    #motionplanner.move(poses["du"]["matrix"])

    sleep(5)
    bot.arm.set_trajectory_time(moving_time=2.0)
    bot.arm.go_to_home_pose(moving_time=2.0)
    bot.arm.go_to_sleep_pose(moving_time=2.0)
    motionplanner.destroy_node()
    robot_close()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)