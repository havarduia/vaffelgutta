from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
import rclpy
from rclpy._rclpy_pybind11 import RCLError
import rclpy.executors
from threading import Thread
from time import sleep
from robot.tools.file_manipulation import Jsonreader
import numpy as numphy
import os

def print_jointstate(bot: Wafflebot):
    while True:
        os.system("clear")
        print(bot.motionplanner.joint_states)
        sleep(0.3)
    

def main(bot: Wafflebot):
    Thread(target=print_jointstate,args=(bot,),daemon=True).start()
    bot.go_to_home_pose()
    reader = Jsonreader()
    poses = reader.read("recordings")
    bot.move(poses["zestyheil"]["matrix"])
    sleep(0.5)
    
    bot.go_to_home_pose()
    bot.go_to_sleep_pose()
    

if __name__ == "__main__":
    bot = None
    try:
        bot = Wafflebot()
        main(bot)
        bot.exit()
    except (Exception, KeyboardInterrupt, RCLError) as e:
        handle_error(e, bot)

