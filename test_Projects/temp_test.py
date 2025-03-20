from robot.tools.timekeeper import record_time, read_times
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot

bot = Wafflebot()
record_time("start")

target = ([
    [1.0, 0.0, 0.0, 10.0],
    [0.0, 1.0, 0.0, 10.0],
    [0.0, 0.0, 1.0, 10.0],
    [0.0, 0.0, 0.0, 1.0],
    ])

from time import time
first_time = time()
testcount = 50
for i in range(testcount):
    bot.arm.set_ee_pose_matrix(target, execute = False)
    record_time(f"movement_{i}")
sum_times = time()-first_time
read_times()
print("The average time is:")
print(sum_times/testcount)

bot.exit()