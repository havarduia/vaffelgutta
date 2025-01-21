from robot_workspace.backend_controllers import find_pose_from_matrix
def check_safety(pose_matrix):
    # test for floor collision
    if pose_matrix[2][3] <= 0.03: # If z is below floor height
        pose_matrix[2][3] = 0.03 # Reset to a taller place

    return pose_matrix
