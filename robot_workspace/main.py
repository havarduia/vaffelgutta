from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown   
from backend_controllers import robot_boot_manager

def main():
    use_real_robot =False 
    robot_boot_manager.robot_launch(use_real_robot=use_real_robot)
    bot = InterbotixManipulatorXS(robot_model="vx300s",group_name="arm",gripper_name="gripper",)
    robot_startup()






if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, Exception):
        robot_shutdown()
        try:
            robot_boot_manager.robot_close()
        except NameError:
            print("Error: Program closed without valid PID")