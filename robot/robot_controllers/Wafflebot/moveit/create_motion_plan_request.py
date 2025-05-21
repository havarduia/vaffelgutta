from moveit_msgs.srv import GetMotionPlan
from robot.robot_controllers.Wafflebot.moveit.create_pose_goal_constraints import create_pose_goal_constraints
from robot.robot_controllers.Wafflebot.moveit.create_robot_state import create_robot_state_from_joint_states

def create_motion_plan_request(start_state, goal_state, speed_scaling) -> GetMotionPlan.Request:
    # Boilerplate
    request = GetMotionPlan.Request()
    request.motion_plan_request.group_name = "interbotix_arm"
    request.motion_plan_request.num_planning_attempts = 8
    request.motion_plan_request.allowed_planning_time = 2.0
    request.motion_plan_request.max_acceleration_scaling_factor = min(0.069*speed_scaling, 1.0)
    request.motion_plan_request.max_velocity_scaling_factor = min(0.069*speed_scaling, 1.0)
    # Create robot start state
    request.motion_plan_request.start_state = create_robot_state_from_joint_states(start_state)
    # Create goal constraints
    goal_constraints = create_pose_goal_constraints(goal_state)
    request.motion_plan_request.goal_constraints = [goal_constraints]
    return request
