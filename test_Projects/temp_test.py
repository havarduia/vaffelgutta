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

for i in range(20):
    bot.arm.set_ee_pose_matrix(target, execute = False)
    record_time(f"movement_{i}")
read_times()
