
from moveit_msgs.msg import (
    PositionConstraint,
    OrientationConstraint,
    Constraints
)

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_matrix

def get_pose_goal_from_matrix(homo_matrix) -> PoseStamped:
    # Convert the homogeneous matrix into a PoseStamped.
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"  # Adjust if needed.
    pose_goal.pose.position.x = homo_matrix[0][3]
    pose_goal.pose.position.y = homo_matrix[1][3]
    pose_goal.pose.position.z = homo_matrix[2][3]
    quat = quaternion_from_matrix(homo_matrix)
    pose_goal.pose.orientation.x = quat[0]
    pose_goal.pose.orientation.y = quat[1]
    pose_goal.pose.orientation.z = quat[2]
    pose_goal.pose.orientation.w = quat[3]
    return pose_goal

def get_positionconstraint(pose_goal: PoseStamped) -> PositionConstraint:
    # Create a position constraint.
    pos_constraint = PositionConstraint()
    pos_constraint.header = pose_goal.header
    pos_constraint.link_name = "vx300s/ee_gripper_link"

    circle = SolidPrimitive()
    circle.type = SolidPrimitive.SPHERE
    circle.dimensions = [0.05]  # 5 cm tolerance.
    
    pos_constraint.constraint_region.primitives.append(circle)
    pos_constraint.constraint_region.primitive_poses.append(pose_goal.pose)
    pos_constraint.weight = 1.0

    return pos_constraint

def create_orientation_constraint(pose_goal: PoseStamped) ->OrientationConstraint:
    # Create an orientation constraint.
    ori_constraint = OrientationConstraint()
    ori_constraint.header = pose_goal.header
    ori_constraint.link_name = "vx300s/ee_gripper_link"
    ori_constraint.orientation = pose_goal.pose.orientation
    ori_constraint.absolute_x_axis_tolerance = 0.1
    ori_constraint.absolute_y_axis_tolerance = 0.1
    ori_constraint.absolute_z_axis_tolerance = 0.1
    ori_constraint.weight = 1.0
    return ori_constraint
    

def create_pose_goal_constraints(homo_matrix):
    # Convert the homogeneous matrix into a PoseStamped.
    pose_goal = get_pose_goal_from_matrix(homo_matrix)
    # Get constraints    
    position_constraint = get_positionconstraint(pose_goal)  
    orientation_constraint = create_orientation_constraint(pose_goal)
    # create constraints object
    constraints = Constraints()
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)

    return constraints
