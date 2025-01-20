"""
This script is used to convert a 4x4 pose matrix
into its components.
This conversion allows use of the built-in
set_cartesian_trajectory function that is recommended
by the interbotix website
https://docs.trossenrobotics.com/interbotix_xsarms_docs/python_ros_interface.html#tips-best-practices

This approach was scrapped due to frequent runtime errors
causing the robot to cancel movement.
It is kept as legacy code,
as a surprise tool to help us later 
"""

class Coordinatesystem:
    x = None,
    y = None,
    z = None,
    roll = None,
    pitch = None,
    yaw = None 

"""
Convert matrix to euler angles.
Using code from chatgpt.
full conversation history at end of file
"""
def rotation_matrix_to_euler_angles(R) -> Coordinatesystem:
    import numpy as np
    """
    Converts a 3x3 rotation matrix to Euler angles (roll, pitch, yaw) following the ZYX convention.
    
    Parameters:
        R (numpy.ndarray): A 3x3 rotation matrix.
        
    Returns:
        tuple: (roll, pitch, yaw) in radians.
    """
    # Check for gimbal lock (when pitch is +/-90 degrees)
    if np.abs(R[2, 0]) != 1:
        pitch = np.arcsin(-R[2, 0])  # θ (pitch)
        roll = np.arctan2(R[2, 1], R[2, 2])  # ϕ (roll)
        yaw = np.arctan2(R[1, 0], R[0, 0])  # ψ (yaw)
    else:
        # Gimbal lock occurs
        yaw = 0  # Arbitrarily set yaw to zero
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])

    return roll, pitch, yaw


def find_pose_from_matrix(targetpose_matrix: list, currentpose_matrix: list):
    
    trajectory_matrix = targetpose_matrix-currentpose_matrix

    target_positions = Coordinatesystem()
    import numpy as np
    
    rotation_matrix = np.array([
     [   trajectory_matrix[0][0], trajectory_matrix[0][1], trajectory_matrix[0][2]],
     [   trajectory_matrix[1][0], trajectory_matrix[1][1], trajectory_matrix[1][2]],
     [   trajectory_matrix[2][0], trajectory_matrix[2][1], trajectory_matrix[2][2]],
    ]) 

    # convert rotation matrix to its roll/pitch/yaw components
    (roll, pitch, yaw) = rotation_matrix_to_euler_angles(rotation_matrix)

    # Enter the components in a struct    
    target_positions.x = trajectory_matrix[0][3]
    target_positions.y = trajectory_matrix[1][3]
    target_positions.z = trajectory_matrix[2][3]
    target_positions.roll = yaw 
    target_positions.pitch = pitch 
    target_positions.yaw = roll

    return target_positions



# Message history with ChatGPT (18.01.2024, gpt-4o)
'''
You said:
Convert a 3x3 rotation matrix to euler angles (roll, pitch, yaw)
ChatGPT said:
ChatGPT

To convert a 3×33×3 rotation matrix RR into Euler angles (roll, pitch, yaw), you can use the following formulas:
Euler Angles Convention (ZYX - Yaw, Pitch, Roll)

This assumes a rotation order of ZYX, meaning:

    Yaw (ψ - around Z-axis)
    Pitch (θ - around Y-axis)
    Roll (ϕ - around X-axis)

Given a rotation matrix:
R=[r11r12r13r21r22r23r31r32r33]
R=
​r11​r21​r31​​r12​r22​r32​​r13​r23​r33​​
​

The Euler angles can be extracted as follows:
Yaw (ψ - rotation around Z-axis):
ψ=atan2(r21,r11)
ψ=atan2(r21​,r11​)
Pitch (θ - rotation around Y-axis):
θ=asin(−r31)
θ=asin(−r31​)
Roll (ϕ - rotation around X-axis):
ϕ=atan2(r32,r33)
ϕ=atan2(r32​,r33​)
Special Case: Gimbal Lock

If ∣r31∣=1∣r31​∣=1, then pitch is at ±90∘±90∘ (or ±π2±2π​), causing gimbal lock. In this case:

    Set θ=±π2θ=±2π​.

    Yaw and Roll become coupled, so you can set ψ=0ψ=0 and compute roll as:
    ϕ=atan2(−r12,r22)
    ϕ=atan2(−r12​,r22​)

Would you like a Python function to compute this?
You said:
yes
ChatGPT said:
ChatGPT

Here's a Python function to convert a 3×33×3 rotation matrix to Euler angles (roll, pitch, yaw) using the ZYX convention:

import numpy as np

def rotation_matrix_to_euler_angles(R):
    """
    Converts a 3x3 rotation matrix to Euler angles (roll, pitch, yaw) following the ZYX convention.
    
    Parameters:
        R (numpy.ndarray): A 3x3 rotation matrix.
        
    Returns:
        tuple: (roll, pitch, yaw) in radians.
    """
    # Check for gimbal lock (when pitch is +/-90 degrees)
    if np.abs(R[2, 0]) != 1:
        pitch = np.arcsin(-R[2, 0])  # θ (pitch)
        roll = np.arctan2(R[2, 1], R[2, 2])  # ϕ (roll)
        yaw = np.arctan2(R[1, 0], R[0, 0])  # ψ (yaw)
    else:
        # Gimbal lock occurs
        yaw = 0  # Arbitrarily set yaw to zero
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])

    return roll, pitch, yaw

# Example usage:
R = np.array([
    [0.866, -0.5, 0.0],
    [0.5, 0.866, 0.0],
    [0.0, 0.0, 1.0]
])

roll, pitch, yaw = rotation_matrix_to_euler_angles(R)
print("Roll:", np.degrees(roll), "degrees")
print("Pitch:", np.degrees(pitch), "degrees")
print("Yaw:", np.degrees(yaw), "degrees")

How it Works:

    Checks for gimbal lock (when the pitch is ±90°).
    If no gimbal lock, computes:
        Pitch (θ) using asin(-R[2,0])
        Roll (ϕ) using atan2(R[2,1], R[2,2])
        Yaw (ψ) using atan2(R[1,0], R[0,0])
    If gimbal lock occurs, assigns:
        Pitch to ±90°
        Roll using alternative formula
        Yaw arbitrarily set to 0

Let me know if you need modifications! 🚀
'''