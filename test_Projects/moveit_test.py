#!/usr/bin/env python3
import sys
import time
import subprocess

import rclpy
from rclpy.node import Node
import rclpy.action

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    RobotState,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

import tf_transformations

def launch_moveit_interface():
    command = [
        "ros2",
        "launch",
        "interbotix_xsarm_moveit_interface",
        "xsarm_moveit_interface.launch.py",
        "robot_model:=vx300s",
    ]
    process = subprocess.Popen(command)
    # Wait a few seconds for the interface to initialize.
    time.sleep(5)
    return process

class MotionPlanner(Node):
    def __init__(self, moveit_process):
        super().__init__('vx300s_motion_planner')
        # Store the MoveIt interface process so we can kill it later.
        self.moveit_process = moveit_process

        # Create the planning service client.
        self.planning_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Define joint states for joint-space planning.
        self.sleep_state = {
            "elbow": 1.55,
            "forearm_roll": 0.0,
            "shoulder": -1.76,
            "waist": 0.0,
            "wrist_angle": 0.8,
            "wrist_rotate": 0.0
        }
        self.home_state = {
            "elbow": 0.0,
            "forearm_roll": 0.0,
            "shoulder": 0.0,
            "waist": 0.0,
            "wrist_angle": 0.0,
            "wrist_rotate": 0.0
        }
        # Target pose provided as a 4x4 homogeneous matrix.
        self.target_pose_matrix = [
            [1, 0, 0, 0.5],
            [0, 1, 0, 0.2],
            [0, 0, 1, 0.3],
            [0, 0, 0, 1]
        ]

        self.group_name = "interbotix_arm"
        # Sequence stages:
        # 0 = Sleep → Home (joint planning)
        # 1 = Home → Pose (Cartesian planning using the provided matrix)
        # 2 = Pose → Sleep (joint planning using extracted final joint state)
        self.plan_stage = 0

        # To store the last computed trajectory.
        self.last_trajectory = None

        # Create an action client for trajectory execution.
        self.exec_action_client = rclpy.action.ActionClient(
            self, FollowJointTrajectory, '/vx300s/arm_controller/follow_joint_trajectory'
        )

        self.wait_for_services()
        # Begin the sequence: move from Sleep to Home.
        self.send_planning_request(self.sleep_state, self.home_state)

    def wait_for_services(self):
        while not self.planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for motion planning service...')
        self.get_logger().info('Motion planning service available.')
        self.get_logger().info('Waiting for execution action server...')
        self.exec_action_client.wait_for_server()
        self.get_logger().info('Execution action server available.')

    def create_robot_state(self, state_input):
        if isinstance(state_input, dict):
            state = RobotState()
            js = JointState()
            js.name = list(state_input.keys())
            js.position = list(state_input.values())
            state.joint_state = js
            return state
        else:
            # If not a dict, use the current state.
            return RobotState()

    def create_goal_constraints(self, joint_dict):
        cons = Constraints()
        for joint_name, target_value in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_value
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        return cons

    def create_pose_goal_constraints(self, hom_matrix):
        # Convert the homogeneous matrix into a PoseStamped.
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"  # Adjust if needed.
        pose_goal.pose.position.x = hom_matrix[0][3]
        pose_goal.pose.position.y = hom_matrix[1][3]
        pose_goal.pose.position.z = hom_matrix[2][3]
        quat = tf_transformations.quaternion_from_matrix(hom_matrix)
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.orientation.w = quat[3]

        # Create a position constraint.
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = "vx300s/ee_gripper_link"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05]  # 5 cm tolerance.
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose_goal.pose)
        pos_constraint.weight = 1.0

        # Create an orientation constraint.
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_goal.header
        ori_constraint.link_name = "vx300s/ee_gripper_link"
        ori_constraint.orientation = pose_goal.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        cons = Constraints()
        cons.position_constraints.append(pos_constraint)
        cons.orientation_constraints.append(ori_constraint)
        return cons

    def send_planning_request(self, start_state, goal_state):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = self.group_name
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0

        req.motion_plan_request.start_state = self.create_robot_state(start_state)
        if isinstance(goal_state, list):
            goal_constraints = self.create_pose_goal_constraints(goal_state)
            goal_label = "Pose"
        else:
            goal_constraints = self.create_goal_constraints(goal_state)
            if goal_state == self.sleep_state:
                goal_label = "Sleep"
            elif goal_state == self.home_state:
                goal_label = "Home"
            else:
                goal_label = "Unknown"
        from_label = self.get_state_name(start_state)
        self.get_logger().info(f"Sending planning request from {from_label} to {goal_label}...")
        req.motion_plan_request.goal_constraints = [goal_constraints]

        future = self.planning_client.call_async(req)
        future.add_done_callback(self.planning_response_callback)

    def get_state_name(self, state):
        if isinstance(state, list):
            return "Pose"
        if state == self.sleep_state:
            return "Sleep"
        elif state == self.home_state:
            return "Home"
        else:
            return "Unknown"

    def planning_response_callback(self, future):
        try:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1:
                self.get_logger().info("Motion plan successfully computed!")
                self.last_trajectory = response.motion_plan_response.trajectory.joint_trajectory
                self.execute_trajectory(self.last_trajectory)
            else:
                self.get_logger().warn(
                    f"Motion plan failed with error code: {response.motion_plan_response.error_code.val}"
                )
                self.shutdown_node()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.shutdown_node()

    def execute_trajectory(self, trajectory: JointTrajectory):
        self.get_logger().info("Sending trajectory for execution...")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        send_goal_future = self.exec_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.execution_response_callback)

    def execution_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Trajectory execution goal rejected!")
                self.shutdown_node()
                return
            self.get_logger().info("Trajectory execution goal accepted, waiting for result...")
            goal_handle.get_result_async().add_done_callback(self.execution_result_callback)
        except Exception as e:
            self.get_logger().error(f"Execution goal failed: {e}")
            self.shutdown_node()

    def execution_result_callback(self, future):
        try:
            _ = future.result().result
            self.get_logger().info("Trajectory execution completed.")
        except Exception as e:
            self.get_logger().error(f"Execution result failed: {e}")
        
        # Sequence the planning requests:
        if self.plan_stage == 0:
            self.plan_stage = 1
            self.get_logger().info("Planning next: from Home to Pose")
            self.send_planning_request(self.home_state, self.target_pose_matrix)
        elif self.plan_stage == 1:
            # Extract final joint positions from the executed trajectory.
            final_point = self.last_trajectory.points[-1]
            joint_names = self.last_trajectory.joint_names
            final_joint_state = {name: pos for name, pos in zip(joint_names, final_point.positions)}
            self.plan_stage = 2
            self.get_logger().info("Planning next: from (extracted) Pose to Sleep")
            self.send_planning_request(final_joint_state, self.sleep_state)
        else:
            self.get_logger().info("Completed planning and execution sequence.")
            # Wait 10 seconds with countdown, then kill the MoveIt interface.
            for i in range(10, 0, -1):
                self.get_logger().info(f"Shutting down MoveIt interface in {i} seconds...")
                time.sleep(1)
            self.shutdown_node()

    def shutdown_node(self):
        # Terminate the MoveIt interface process if it's still running.
        if self.moveit_process.poll() is None:
            self.get_logger().info("Terminating MoveIt interface process...")
            self.moveit_process.terminate()
        self.destroy_node()
        sys.exit(0)

def main(args=None):
    moveit_proc = launch_moveit_interface()
    rclpy.init(args=args)
    node = MotionPlanner(moveit_process=moveit_proc)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to keyboard interrupt...")
    finally:
        if rclpy.ok():
            node.shutdown_node()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
