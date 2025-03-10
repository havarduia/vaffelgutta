from time import sleep
from robot.robot_controllers.Wafflebot import Wafflebot
from random import randint

def throw_rock(bot: Wafflebot):
    bot.arm.set_ee_cartesian_trajectory(roll=3.141592/2)

def throw_paper(bot: Wafflebot):    
    bot.gripper.release()
    

def throw_scissors(bot: Wafflebot):
    bot.gripper.release()
    bot.arm.set_ee_cartesian_trajectory(roll=3.141592/2)

def rock_paper_scissors(bot: Wafflebot):
    bot.arm.go_to_home_pose()
    bot.gripper.grasp()
    for i in range(3):
        bot.arm.set_joint_positions
        bot.arm.set_single_joint_position("shoulder", -0.2, moving_time=0.5)
        bot.arm.set_single_joint_position("elbow", -0.3, moving_time= 0.5)
        bot.arm.set_single_joint_position("shoulder", 0.0, moving_time=0.5)
        bot.arm.set_single_joint_position("elbow", -0.0, moving_time=0.5)
        
    bot.arm.set_single_joint_position("elbow", -0.0, moving_time=0.5)
    
    bot.arm.set_trajectory_time(moving_time=2.0)
        
    action = randint(1,3)
    if action == 1:
        throw_rock(bot=bot)
    elif action == 2:
        throw_paper(bot=bot)
    elif action == 3:
        throw_scissors(bot=bot)
    sleep(2.5)
    bot.arm.go_to_home_pose()
    bot.gripper.grasp()
        