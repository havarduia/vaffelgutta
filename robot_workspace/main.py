from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown   
import backend_controllers.robot_boot_manager as boot_manager

def main():
    use_real_robot =False 
    pid = boot_manager.robot_launch(use_real_robot=use_real_robot)
    bot = InterbotixManipulatorXS(robot_model="vx300s",group_name="arm",gripper_name="gripper",)
    robot_startup()
    try:





    except KeyboardInterrupt:
        robot_shutdown()
        boot_manager.robot_close(pid)





if __name__ == '__main__':
    main()