from time import sleep
def safe_stop(bot:"Wafflebot", slow):
    home_pose = bot.arm.robot_des.M
    sleep_pose = (
        [[ 0.83094068, 0.,          0.55636102,  0.13459297],
        [ 0.,          1.,          0.,          0.,        ],
        [-0.55636102,  0.,          0.83094068,  0.09320746],
        [ 0.,          0.,          0.,          1.        ]]) 
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.set_trajectory_time(moving_time=(8.0 if slow else 2.0)) # reset moving time if changed elsewhere
    sleep(2)
    bot.arm.go_to_home_pose()
    sleep_joints =  [0.0, -1.80, 1.6, 0.0, 0.5859, 0.0]
    bot.arm.set_joint_positions(sleep_joints)
    sleep(0.5)

if __name__ == "__main__":
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot