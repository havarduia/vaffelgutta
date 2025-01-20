from time import sleep
from robot_workspace.assets.Wafflebot import Wafflebot
from random import randint

def throw_rock(bot: Wafflebot):
    bot.arm.set_ee_cartesian_trajectory(roll=3.141592/2)

def throw_paper(bot: Wafflebot):    
    bot.gripper.release()
    

def throw_scissors(bot: Wafflebot):
    bot.gripper.release()
    bot.arm.set_ee_cartesian_trajectory()

def rock_paper_scissors(bot: Wafflebot):
    bot.arm.go_to_home_pose()
    bot.gripper.grasp()
    for i in range(3):
        bot.arm.set_single_joint_position("arm", 1)
        bot.arm.set_single_joint_position("arm", 0)
    action = randint(1,3)
    if action == 1:
        throw_rock(bot=bot)
    elif action == 2:
        throw_paper(bot=bot)
    elif action == 3:
        throw_scissors(bot=bot)
    bot.arm.go_to_home_pose()
    bot.gripper.grasp()
        