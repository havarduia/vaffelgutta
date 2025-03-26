#!/usr/bin/env python3
from time import sleep
from robot.robot_controllers.robot_boot_manager import robot_launch, robot_close
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot.moveit.create_motion_plan_request import create_motion_plan_request
from robot.robot_controllers.Wafflebot.moveit.create_collisionobjects import CollisionObjectPublisher
from robot.tools.file_manipulation import Jsonreader

import rclpy
from rclpy.node import Node
import rclpy.action
import rclpy.subscription

from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class MotionPlanner(Node):
    def __init__(self, interbotix_moveit_process):
        #name node:
        super().__init__('vx300s_motion_planner')
        # Store Process of interbotix (TODO add this feature to wafflebot)
        self.interbotix_moveit_process = interbotix_moveit_process
        self.group_name = "interbotix_arm"
        # Create ros clients
        self.planning_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.exec_action_client = rclpy.action.ActionClient(
            self, FollowJointTrajectory, '/vx300s/arm_controller/follow_joint_trajectory'
        )
        self.joint_state_subscriber = self.create_subscription(
            JointState, "/vx300s/joint_states", self._update_joint_state, 10
            )
        # Check if clients have loaded successfully
        if not self.planning_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError ("Planning service would not load. Please restart. If problem persists, please reboot.")
        if not self.exec_action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError ("Execution service would not load. Please restart. If problem persists, please reboot.")
        
        self.moving = False
        self.target_pose_matrix = None
        self.joint_states = None
        self.gripper_state = None
        self.update_count = 0
        self.movement_success =False 
        

    def update_joint_states(self):
        update_count = self.update_count
        while update_count == self.update_count:
            rclpy.spin_once(self)
        return self.joint_states

    def _update_joint_state(self, joint_states: JointState):
        self.update_count +=1
        # nip overflow errors in the bud
        if self.update_count > 10_000:
            self.update_count = 0    
        self.joint_states = list(joint_states.position)[:6]
        self.gripper_state = list(joint_states.position)[-1]
        return

    def update_collisionobjects(self, ignore):
        reader = Jsonreader()
        reader.update_filedirectory("robot/assets/boundingboxes/")
        collisionobjects: dict = reader.read("boundingboxes_static")
        collisionobjects.update(reader.read("boundingboxes_dynamic"))
        publisher = CollisionObjectPublisher()
        publisher.publish_collisionobjects(collisionobjects, ignore)

    def move(self, 
             target: list[list[float]],
               speed_scaling: float = 1.0,
              ignore: list[str] = None
              ) -> None: 
        if self.moving:
            print("already moving")
            self.movement_success = False
        self.moving = True
        self.update_collisionobjects(ignore)
        self.update_joint_states()
        start_state = self.joint_states
   
        self.planning_request(start_state, target, speed_scaling)
        while self.moving:
            rclpy.spin_once(self)

    def planning_request(self, start_state, goal_state, speed_scaling):
        mp_request = create_motion_plan_request(start_state, goal_state, speed_scaling) 
        motionplan_future = self.planning_client.call_async(mp_request)
        motionplan_future.add_done_callback(self.planning_callback)

    def planning_callback(self, future):
        try:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1: # Error code 1 means success. Anything else is a failure
                target_trajectory = response.motion_plan_response.trajectory.joint_trajectory
                self.execute_trajectory_request(target_trajectory)
            else:
                self.moving = False
                self.movement_success = False
                return False
        except Exception as e:
            self.moving = False
            self.movement_success = False
            print("Motion planner broke UwU")
            print(e.with_traceback())
            return "Motion planner broke UwU" 
    

    def execute_trajectory_request(self, trajectory: JointTrajectory):
        # Sends ROS request to execute the planned trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        send_goal_future = self.exec_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.execute_trajectory_callback)

    def execute_trajectory_callback(self, future):
        # if path approved, prepare trajectory finished. Else assume world has burned down.
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.moving = False
                self.movement_success = False
                return False
            goal_handle.get_result_async().add_done_callback(self.trajectory_finished_callback)
        except Exception as e:
            self.moving = False
            self.movement_success = False
            print(e.with_traceback())
            return "Motion planner broke UwU" 
            
    def trajectory_finished_callback(self, future):
        try:
            _ = future.result().result
            self.moving = False
            self.movement_success = True
            return True
        except Exception as e:
            self.movement_success = False
            self.moving = False
            print(f"Execution result failed: {e}")
    
    def __del__(self):
        self.shutdown_node()

    def shutdown_node(self):
        # Terminate the MoveIt interface process if it's still running.
        if self.interbotix_moveit_process.poll() is None:
            self.interbotix_moveit_process.terminate()
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


