from robot.tools.timekeeper import record_time, read_times
from time import sleep
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from os import system
from modern_robotics import FKinSpace
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

bot = InterbotixManipulatorXS(robot_model="vx300s")#Wafflebot(use_real_robot=False, use_rviz= False)

system("clear")
sleep_state = [0.0, -1.76, 1,55,0.0,0.8,0.0] 
home_state = [0.0]*6
print(home_state)
pose = FKinSpace(bot.bot.arm.robot_des.M, bot.bot.arm.robot_des.Slist, sleep_state) 

print(pose)

pose = FKinSpace(bot.bot.arm.robot_des.M, bot.bot.arm.robot_des.Slist, sleep_state) 

print(pose)