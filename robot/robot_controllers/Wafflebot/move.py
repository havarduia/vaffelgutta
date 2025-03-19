from time import sleep
from robot.robot_controllers.path_planner import *


def move(bot: "InterbotixManipulatorXS",
        target_joints: list[float], 
        ignore: list[str] = None,
        blocking: bool = True, 
        speed_scaling: float = 1.0, 
        debug_print: bool = False
        ) -> None:
    
    start_joints = bot.arm.get_joint_positions()
    speedconstant = 0.42066638
    min_move_time = 0.314159265
    
    waypoints, success = (plan_path(bot, start_joints, target_joints, ignore))

    if not success: 
        if debug_print:
            print("Wafflebot.move: move failed after path planner")
        return False
    
    prev_waypoint = start_joints
    for waypoint in waypoints:
        joint_travel_distance =list_sum(waypoint, list_multiply(prev_waypoint,-1)) 
        wp_path_length = max(calculate_biggest_joint(joint_travel_distance), 1e-8)
        speed = (speedconstant * speed_scaling / wp_path_length) 
        move_time = max(1.0/speed, min_move_time)
        bot.arm.set_joint_positions(waypoint,move_time,move_time/2,False)
        prev_waypoint = waypoint
        if blocking:
            # todo make interruptible. (e. stop etc)
            sleep(move_time)
        else:
            sleep(min_move_time)

if __name__ == "__main__":
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS