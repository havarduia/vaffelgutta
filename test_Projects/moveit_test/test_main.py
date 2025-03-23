from test_Projects.moveit_test.new_moveit_test import MotionPlanner
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot.robot_controllers.robot_boot_manager import robot_launch
from robot.tools.errorhandling import handle_error
import rclpy
from threading import Thread
from time import sleep
from robot.tools.file_manipulation import Jsonreader

def main():
    bot = InterbotixManipulatorXS(robot_model="vx300s")
    rclpy.init()
    interbotix_moveit_process = robot_launch(use_real_robot=0)
    motionplanner = MotionPlanner(bot, interbotix_moveit_process)
    Thread(rclpy.spin, daemon=True, args=(motionplanner,)).start()
    sleep(5)
    reader = Jsonreader()
    poses = reader.read("recordings")
    
    motionplanner.move(poses["matrix"][])
    sleep(2)
    motionplanner.move(poses["matrix"][])

    sleep(5)
    bot.arm.go_to_home_pose()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)