from robot_workspace.backend_controllers import find_pose_from_matrix
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
def check_safety(bot: InterbotixManipulatorXS, pose_matrix):
    
    # test for floor collision
    if pose_matrix[2][3] <= 0.03: # If z is below floor height
        pose_matrix[2][3] = 0.03 # Reset to a taller place
        
    return pose_matrix
