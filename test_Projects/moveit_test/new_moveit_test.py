#!/usr/bin/env python3
import asyncio
from time import sleep
from robot.robot_controllers.robot_boot_manager import robot_launch, robot_close
from robot.tools.errorhandling import handle_error
#Håvard:
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

from test_Projects.moveit_test.create_pose_goal_constraints import create_pose_goal_constraints
from test_Projects.moveit_test.create_motion_plan_request import create_motion_plan_request

from test_Projects.moveit_test.create_robot_state import create_robot_state
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import tf_transformations

class MotionPlanner(Node):
    def __init__(self, bot: "InterbotixManipulatorXS", interbotix_moveit_process):
        #name node:
        super().__init__('vx300s_motion_planner')
        # Store Process of interbotix (TODO add this feature to wafflebot)
        self.interbotix_moveit_process = interbotix_moveit_process
        self.group_name = "interbotix_arm"
        self.bot = bot
        # Create ros clients
        self.planning_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.exec_action_client = rclpy.action.ActionClient(
            self, FollowJointTrajectory, '/vx300s/arm_controller/follow_joint_trajectory'
        )
        # Check if clients have loaded successfully
        if not self.planning_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError ("Planning service would not load. Please restart. If problem persists, please reboot.")
        if not self.exec_action_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError ("Execution service would not load. Please restart. If problem persists, please reboot.")

        self.target_pose_matrix = None
        self.moving = False
        
    def move(self, target): 
        if self.moving:
            print("already moving")
            return False
        start_state = self.bot.arm.get_joint_positions()
        self.planning_request(start_state, target)

    def planning_request(self, start_state, goal_state):
        mp_request = create_motion_plan_request(start_state, goal_state) 
        motionplan_future = self.planning_client.call_async(mp_request)
        motionplan_future.add_done_callback(self.planning_callback)

    def planning_callback(self, future):
        if not self.moving:
            try:
                response = future.result()
                if response.motion_plan_response.error_code.val == 1: # Error code 1 means success. Anything else is a failure
                    target_trajectory = response.motion_plan_response.trajectory.joint_trajectory
                    self.execute_trajectory_request(target_trajectory)
                    return True
                else:
                    return False
            except Exception as e:
                print("Motion planner broke UwU")
                print(e.with_traceback())
                return "Motion planner broke UwU" 
        else:
            print("Bruh moveit")

    def execute_trajectory_request(self, trajectory: JointTrajectory):
        # Sends ROS request to execute the planned trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        self.moving = True
        send_goal_future = self.exec_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.execute_trajectory_callback)

    def execute_trajectory_callback(self, future):
        # if path approved, prepare trajectory finished. Else assume world has burned down.
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.moving = False
                return False
            goal_handle.get_result_async().add_done_callback(self.trajectory_finished_callback)
        except Exception as e:
            print(e.with_traceback())
            return "Motion planner broke UwU" 
            
    def trajectory_finished_callback(self, future):
        try:
            _ = future.result().result
            self.moving = False
            return True
        except Exception as e:
            print(f"Execution result failed: {e}")
    

    def shutdown_node(self):
        # Terminate the MoveIt interface process if it's still running.
        if self.moveit_process.poll() is None:
            self.moveit_process.terminate()
        self.destroy_node()



def main():

    rclpy.init()
    interbotix_moveit_process = robot_launch(use_real_robot=0)
    node = MotionPlanner(interbotix_moveit_process)
    sleep(6)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to keyboard interrupt...")
    finally:
        if rclpy.ok():
            node.shutdown_node()
            node.destroy_node()
            rclpy.shutdown()

    return




if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)
    finally:
        if rclpy.ok():
            robot_close()