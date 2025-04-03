from robot.robot_controllers.Wafflebot import *
from robot.robot_controllers import robot_boot_manager
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import rclpy
from time import sleep
from robot.robot_controllers.path_planner import list_multiply, list_sum
from robot.robot_controllers.Wafflebot.moveit.MotionPlanner import MotionPlanner
from camera.coordinatesystem import CoordinateSystem
from robot.robot_controllers.Wafflebot.add_collisionobjects import add_collisionobjects

class Wafflebot:
    def __init__(
        self,
        cam: CoordinateSystem,
        debug_print: bool = False,
        use_rviz: bool = True,
    ):
        # Initialize robot:
        interbotix_process = robot_boot_manager.robot_launch(use_rviz)
        self.bot = InterbotixManipulatorXS(
            robot_model="vx300s",
            group_name="arm",
            gripper_name="gripper",
        )
        self.motionplanner = MotionPlanner(interbotix_process)
        robot_startup()

        # Keep a look out for the emergency stop
        self.launch_emergency_stop_monitor()
        # Define shorthands to call bot functions intuitively
        self.arm = self.bot.arm
        self.gripper = self.bot.gripper
        self.core = self.bot.core
        # misc inits
        self.home_pose = self.arm.robot_des.M
        self.debug_print = debug_print
        self.speed = 1.0
        self.cam = cam

        # initialize joint positions
        self.motionplanner.update_joint_states()

    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)

    def go_to_home_pose(self):
        self.move(self.arm.robot_des.M)
        start_joints = self.motionplanner.update_joint_states()
        for i in range(1, 51):
            self.arm.set_joint_positions(list_multiply(start_joints, (1 - i / 50)), blocking=False)
            sleep(0.02)
        sleep(0.2)

    def go_to_sleep_pose(self):
        start_joints = self.motionplanner.update_joint_states()
        sleep_joints = [0.0, -1.80, 1.59, 0.0, 0.5959, 0.0]
        for i in range(1, 51):
            # sleep = start + (start-sleep)*T/dt
            self.arm.set_joint_positions(
                list_sum(
                    start_joints,
                    list_multiply(
                        list_sum(sleep_joints, list_multiply(start_joints, -1)),
                        (i/50),
                    ),
                ),
            blocking=False
            )
            sleep(0.012)
        sleep(0.4)

    def exit(self):
        if rclpy.ok():
            self.motionplanner.destroy_node()
            robot_shutdown()
            robot_boot_manager.robot_close()
            if rclpy.ok():
                rclpy.shutdown()

    def safe_stop(self, slow=False):
        if slow:
            self.move(self.home_pose, speed_scaling=0.1)
            self.go_to_home_pose()
            self.go_to_sleep_pose()
        else:
            self.go_to_home_pose()
            self.go_to_sleep_pose()
        self.exit()

    def move(self, target, ignore: list[str]=None, speed_scaling: float = 1.0):
        self.cam.start("all")
        success = add_collisionobjects(ignore)
        if success:
            self.motionplanner.move(target, speed_scaling)
            return self.motionplanner.movement_success
        elif self.debug_print:
            print("Wafflebot: Collision publishing failed")
    def launch_emergency_stop_monitor(self):
        emergency_stop.run_emergency_stop_monitor(self.safe_stop)
