from moveit_msgs.srv import GetMotionPlan
from test_Projects.moveit_test.create_pose_goal_constraints import create_pose_goal_constraints
from test_Projects.moveit_test.create_robot_state import create_robot_state_from_joint_states


def create_motion_plan_request(start_state, goal_state) -> GetMotionPlan.Request:
    # Boilerplate
    request = GetMotionPlan.Request()
    request.motion_plan_request.group_name = "interbotix_arm"
    request.motion_plan_request.num_planning_attempts = 10
    request.motion_plan_request.allowed_planning_time = 5.0
    # Create robot start state
    request.motion_plan_request.start_state = create_robot_state_from_joint_states(start_state)
    # Create goal constraints
    goal_constraints = create_pose_goal_constraints(goal_state)
    request.motion_plan_request.goal_constraints = [goal_constraints]
