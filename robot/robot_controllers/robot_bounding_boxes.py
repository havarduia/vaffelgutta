""" functions to generate bounding boxes"""
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy
from os import getcwd

from robot.tools.file_manipulation import Jsonreader

def _bb_from_endpoints(minimum, maximum):
    (xm,ym,zm) = minimum
    (xp,yp,zp) = maximum
    return[
    [xm,ym,zm], 
    [xm,ym,zp],
    [xm,yp,zm],
    [xm,yp,zp],
    [xp,ym,zm],
    [xp,ym,zp],
    [xp,yp,zm],
    [xp,yp,zp]
    ]


def _endpoints_from_bb(corners):
    minimum = [1e4, 1e4, 1e4]
    maximum = [-1e4,-1e4,-1e4]
    # make sure the endpoints are the max of the rotated endpoints for every iteration and axis
    for corner in corners:
        if corner[0] < minimum[0]: minimum[0] = corner[0]
        if corner[1] < minimum[1]: minimum[1] = corner[1]
        if corner[2] < minimum[2]: minimum[2] = corner[2]
        if corner[0] > maximum[0]: maximum[0] = corner[0]
        if corner[1] > maximum[1]: maximum[1] = corner[1]
        if corner[2] > maximum[2]: maximum[2] = corner[2]
    return(minimum, maximum)


def _yaw_point(point, angle, origin): # origin constant = 0
        x0 = origin[0]
        y0 = origin[1]
        x = point[0]
        y = point[1]
        # z = point[2]
        radius = numphy.hypot((x-x0), (y-y0))
        theta_0 = numphy.arctan2((y-y0), (x-x0))  
        point[0] = x0 + float(radius * numphy.cos(theta_0 + angle))
        point[1] = y0 + float(radius * numphy.sin(theta_0 + angle))
        #point[2] = point[2]
        return point

def _yaw_cube(corners, angle, origin):
    for corner in corners:
        corner = _yaw_point(corner, angle, origin)
    return corners

def _pitch_point(point, angle, origin):
    x0 = origin[0]
    #y0 = origin[1]
    z0 = origin[2]
    x = point[0]
    #y = corner[1]
    z = point[2]
    radius = numphy.hypot((x-x0),(z-z0))
    theta_0 = numphy.arctan2((z-z0), (x-x0))
    point[0] = x0+float(radius * numphy.cos(theta_0 + angle))
    #corner[1] = corner[1]
    point[2] = z0+float(radius * numphy.sin(theta_0 + angle))
    return point

def _pitch_cube(corners, angle, origin):
    for corner in corners:
        _pitch_point(corner, angle, origin)
    return corners

def _roll_point(point, angle, origin):
    #x0 = origin[0]
    y0 = origin[1]
    z0 = origin[2]
    #x = point[0]
    y = point[1]
    z = point[2]
    radius = numphy.hypot((y-y0),(z-z0))
    theta_0 = numphy.arctan2((z-z0), (y-y0))  
    #point[0] = point[0]
    point[1] = y0+float(radius * numphy.cos(theta_0 + angle))
    point[2] = z0+float(radius * numphy.sin(theta_0 + angle))
    return point

def _roll_cube(corners, angle, origin):    
    for corner in corners:
        corner = _roll_point(corner, angle,origin)
    return corners

def update_robot_bounding_box(
        joints: list
        ) -> None:
    bounding_boxes = {}
    base_origin = [0.0,0.0,0.0]

    if len(joints) != 6:
        print("Error in bounding boxes -> robot_bounding box:\n"
              +"Please input a list of joint positions")
        return
    
    joint_lengths = [
        4.5e-2, # base to shoulder
        30e-2,  # shoulder to arm_link z
        6.5e-2, # shoulder to arm_link x
        14e-2,  #arm link to motor 
        16e-2,  # motor to wrist
        6.5e-2  # wrist to gripper
        ] # convert cm to m

    "Define rotation angles"
    waist_angle = joints[0]
    shoulder_angle = joints[1]
    elbow_angle = joints[2]
    forearm_roll = joints[3]
    wrist_angle = joints[4]
    gripper_roll = joints[5]
    
    "Shoulder:"
    # Set shoulder position
    shoulder_origin = base_origin
    shoulder_origin[2] += 0.125 # hand measured - distance from world origin to shoulder center

    shoulder_dimensions = [4.5e-2,9e-2,6e-2] # measurements of shoulder fixture,shoulder_min,shoulder_max
    shoulder_offset = [- elem/2 for elem in shoulder_dimensions] # initialize with origin in center
    shoulder_offset[2] = -4.5e-2 # z component measured by hand
    # Determine corners
    
    shoulder_min =[ shoulder_origin[i]+shoulder_offset [i] for i in range(3)]# minimum point is equal to offset
    
    shoulder_max =[ shoulder_min [i]+ shoulder_dimensions [i] for i in range(3)]# define max point of bocxs

    #calculate bounding box
    shoulder_corners = _bb_from_endpoints(shoulder_min, shoulder_max) 
    shoulder_corners = _yaw_cube(shoulder_corners,waist_angle, base_origin)
    (shoulder_min,shoulder_max) = _endpoints_from_bb(shoulder_corners)
    bounding_boxes["shoulder"] = (shoulder_min,shoulder_max)

    "First arm joint"
    # Declare arm dimensions
    arm_1_origin = shoulder_origin.copy()
    arm_1_dimensions = [3e-2, 10e-2, 6e-2] # hand measurements
    arm_1_offset = [-elem/2 for elem in arm_1_dimensions] # x, y offsets are half of the arm size
    arm_1_offset[2] = 0 # the motor is the origin from the z axis
    # Determine corners
    arm_1_min =[ arm_1_origin[i]+arm_1_offset[i] for i in range(3)]
    arm_1_max =[ arm_1_min[i]+arm_1_dimensions[i] for i in range(3)]# locate endpoints
    #arm_1_angle = waist_direction
    
    arm_1_corners = _bb_from_endpoints(arm_1_min, arm_1_max) 
    arm_1_corners = _pitch_cube(arm_1_corners,-shoulder_angle,arm_1_origin)
    arm_1_corners = _yaw_cube(arm_1_corners,waist_angle, base_origin)
    (arm_1_min,arm_1_max) = _endpoints_from_bb(arm_1_corners)
    
    bounding_boxes["arm_1"] = (arm_1_min.copy(), arm_1_max.copy())
    
    
    # measured dimensions:
    arm_1_division_origin = arm_1_origin.copy()
    arm_1_division_dimensions = [2e-2, 4e-2,33e-2]
    arm_1_division_offset = [-elem/2 for elem in arm_1_division_dimensions] # x, y offsets are half of the arm size
    arm_1_division_offset[2] = 0 # z is aligned with origin
    arm_1_division_min=[arm_1_division_origin[i] + arm_1_division_offset[i] for i in range(3)]
    
    # Iterate over multiple segments of the arm to reduce error.
    arm_subdivisions = 10
    DT = arm_subdivisions
    for dt in range(arm_subdivisions): 
        #initialize arm positions
        subdivision_start = arm_1_division_min.copy(); subdivision_end = arm_1_division_min.copy()

        subdivision_end[1] += arm_1_division_dimensions[1] #the width (y) of arm1 is always 4cm
        #create subdivisions
        subdivision_start[0] = arm_1_division_min[0] # start x
        subdivision_start[2] = arm_1_division_min[2]+arm_1_division_dimensions[2]*dt/DT  # start z
        subdivision_end[0] = arm_1_division_min[0]+ arm_1_dimensions[0] # arm 1 ends at an offset exual to its own size
        subdivision_end[2] = arm_1_division_min[2]+arm_1_division_dimensions[2]*(dt+1)/DT# end z
        # create a cube and apply rotation
        
        subdivision_corners = _bb_from_endpoints(subdivision_start,subdivision_end)
        subdivision_corners = _pitch_cube(subdivision_corners, -shoulder_angle, arm_1_division_origin)
        subdivision_corners = _yaw_cube(subdivision_corners, waist_angle, base_origin)
        (subdivision_start, subdivision_end) = _endpoints_from_bb(subdivision_corners)
        
        #save results:
        bounding_boxes[f"arm_1_box_{(dt+1)}"] = (subdivision_start.copy(), subdivision_end.copy()) 


    "Arm link"
    # origin set at the dynamixel servo to arm 2
    arm_link_origin = shoulder_origin.copy()
    arm_link_origin[2] += joint_lengths[1] # raise z by the arm length

    # Calculate offsets to the lowermost point
    arm_link_dimensions =[8.5e-2,10e-2,5e-2] #measured by hand 
    arm_link_offset = [-elem/2 for elem in arm_link_dimensions]
    arm_link_offset[0] = -1e-2 # measured by hand
    arm_link_min =[ arm_link_origin [i]+ arm_link_offset[i] for i in range(3)]
    arm_link_max =[ arm_link_min [i]+ arm_link_dimensions[i] for i in range(3)]# rotate the bounding box
    
    arm_link_corners = _bb_from_endpoints(arm_link_min, arm_link_max)
    arm_link_corners = _pitch_cube(arm_link_corners, -shoulder_angle, shoulder_origin)
    arm_link_corners = _yaw_cube(arm_link_corners, waist_angle, base_origin)
    (arm_link_min, arm_link_max) = _endpoints_from_bb(arm_link_corners)
    
    bounding_boxes["arm_link"] = (arm_link_min, arm_link_max)

    "second arm mount"
    # origin set at the dynamixel servo to arm 2
    arm_2_mount_origin = arm_link_origin.copy() 
    arm_2_mount_origin[0] += 6e-2
    arm_2_mount_origin = _pitch_point(arm_2_mount_origin, -shoulder_angle, shoulder_origin)
    arm_2_mount_origin = _yaw_point(arm_2_mount_origin, waist_angle, shoulder_origin)
    # Calculate offsets to the lowermost point
    arm_2_mount_dimensions =[7e-2,10.5e-2,3.5e-2] #measured by hand 
    arm_2_mount_offset = [-elem/2 for elem in arm_2_mount_dimensions]
    arm_2_mount_offset[0] = 0.0e-2 # distance to start of the joint
    arm_2_mount_min =[ arm_2_mount_origin [i]+ arm_2_mount_offset[i] for i in range(3)]
    arm_2_mount_max =[ arm_2_mount_min [i]+ arm_2_mount_dimensions[i] for i in range(3)]# rotate the bounding box
    arm_2_mount_corners = _bb_from_endpoints(arm_2_mount_min, arm_2_mount_max)
    arm_2_mount_corners = _pitch_cube(arm_2_mount_corners, -shoulder_angle, arm_2_mount_origin)
    arm_2_mount_corners = _pitch_cube(arm_2_mount_corners, -elbow_angle, arm_2_mount_origin)
    arm_2_mount_corners = _yaw_cube(arm_2_mount_corners, waist_angle, arm_2_mount_origin)
    (arm_2_mount_min, arm_2_mount_max) = _endpoints_from_bb(arm_2_mount_corners)

    bounding_boxes["arm_2_mount"] = (arm_2_mount_min, arm_2_mount_max)
    
    "second arm joint"
    # Declare arm dimensions
    arm_2_origin = arm_2_mount_origin.copy()
    arm_2_dimensions = [20e-2, 6e-2,4e-2] # measured by hand
    arm_2_offset = [-elem/2 for elem in arm_2_dimensions]  # y, z offsets are half of the arm size
    arm_2_offset[0] = 0 # the motor is the origin from the x axis
    # Determine corners
    arm_2_min =[ arm_2_origin[i]+ arm_2_offset[i] for i in range(3)]
    #arm_2_angle = waist_direction
    
    # Iterate over multiple segments of the arm to reduce error.
    arm_subs = 5 
    DT = arm_subs
    for dt in range(arm_subs): 
        #initialize arm positions
        sub_start = arm_2_min.copy(); sub_end = arm_2_min.copy()
        sub_end[1] += arm_2_dimensions[1] #the width (y) of arm2 is always 6cm
        #create subs
        sub_start[0] +=arm_2_dimensions[0]*dt/DT     # start x
        #sub_start[2] = arm_2_min[2]  # start z
        sub_end[0] +=(arm_2_dimensions[0]*(dt+1)/DT)   # end x
        sub_end[2] += arm_2_dimensions[2] # arm 2 ends at an offset equal to its own size
        # create a cube and apply rotation

        sub_corners = _bb_from_endpoints(sub_start,sub_end)
        sub_corners = _pitch_cube(sub_corners, -shoulder_angle, arm_2_origin)
        sub_corners = _pitch_cube(sub_corners, -elbow_angle, arm_2_origin)
        sub_corners = _yaw_cube(sub_corners, waist_angle, arm_2_origin)
        (sub_start, sub_end) = _endpoints_from_bb(sub_corners)
        #save results:
        bounding_boxes[f"arm_2_box_{(dt+1)}"] = (sub_start.copy(), sub_end.copy()) 
    
    "motor"
    # Declare arm dimensions
    motor_origin = arm_2_origin.copy()
    motor_origin[0] += 19e-2 # Also measured by hand - the distance from the elbow joint to the beginning of the motor
    motor_origin = _pitch_point(motor_origin, -elbow_angle, arm_2_origin)
    motor_origin = _pitch_point(motor_origin, -shoulder_angle, arm_2_origin)
    motor_origin = _yaw_point(motor_origin, waist_angle, arm_2_origin)
    motor_dimensions = [13e-2,9e-2,4.5e-2] # measured by hand
    motor_offset = [-elem/2 for elem in motor_dimensions]  # y, z offsets are half of the arm size
    motor_offset[0] = 0 # motor origin aligned with axis or sumn
    # Determine corners
    motor_min = [motor_origin[i] + motor_offset[i] for i in range(3)]
    motor_max =[motor_min[i]+ motor_dimensions[i] for i in range(3)]
    #motor_angle = waist_direction
    motor_corners = _bb_from_endpoints(motor_min, motor_max)
    motor_corners = _roll_cube(motor_corners, forearm_roll, motor_origin)
    motor_corners = _pitch_cube(motor_corners,-elbow_angle, motor_origin)
    motor_corners = _pitch_cube(motor_corners, -shoulder_angle, motor_origin)
    motor_corners = _yaw_cube(motor_corners, waist_angle, motor_origin)
  
    (motor_min, motor_max) = _endpoints_from_bb(motor_corners)
 
    bounding_boxes["motor"] = (motor_min.copy(), motor_max.copy())
    
         
    "wrist"
    # Declare arm dimensions
    wrist_origin = arm_2_origin.copy()
    wrist_origin[0] += 30e-2 # the distance between shoulder and wristshoulder_origin
    
    #wrist_origin = _roll_point(wrist_origin, forearm_roll, arm_2_origin)
    wrist_origin = _pitch_point(wrist_origin,-shoulder_angle, arm_2_origin)
    wrist_origin = _pitch_point(wrist_origin,-elbow_angle, arm_2_origin)
    wrist_origin = _yaw_point(wrist_origin, waist_angle, arm_2_origin)

    wrist_dimensions = [7e-2,6e-2,7e-2] # measured by hand, and using technical drawing on interbotix docs website for reference
    wrist_offset = [-elem/2 for elem in wrist_dimensions] # y offsets are half of the arm size
    wrist_offset[0] = 2e-2 # Also measured by hand - the distance from the elbow joint to the beginning of the wrist
    wrist_offset[2] +=1e-2 # a tiny offset to account for the wrists curve upwards 
    # Determine corners
    wrist_min =[ wrist_origin [i]+ wrist_offset  [i] for i in range(3)]
    wrist_max =[ wrist_min [i]+ wrist_dimensions[i] for i in range(3)]

    # Transform coordinate systems
    wrist_corners = _bb_from_endpoints(wrist_min, wrist_max)
    wrist_corners = _pitch_cube(wrist_corners, -wrist_angle,wrist_origin)
    wrist_corners = _roll_cube(wrist_corners, forearm_roll, wrist_origin)
    wrist_corners = _pitch_cube(wrist_corners, -elbow_angle, wrist_origin)
    wrist_corners = _pitch_cube(wrist_corners, -shoulder_angle, wrist_origin)
    wrist_corners = _yaw_cube(wrist_corners, waist_angle, wrist_origin)
    (wrist_min, wrist_max) = _endpoints_from_bb(wrist_corners)

    bounding_boxes["wrist"] = (wrist_min, wrist_max)

    "gripper"
    # Declare arm dimensions
    gripper_origin = wrist_origin.copy()
    gripper_origin[0] += 6e-2 # the distance between- i dont even know anymore
    
    gripper_origin = _pitch_point(gripper_origin, -wrist_angle, wrist_origin)
    gripper_origin = _roll_point(gripper_origin, forearm_roll, wrist_origin)
    gripper_origin = _pitch_point(gripper_origin, -shoulder_angle, wrist_origin)
    gripper_origin = _pitch_point(gripper_origin, -elbow_angle, wrist_origin)
    gripper_origin = _yaw_point(gripper_origin, waist_angle, wrist_origin)

    gripper_dimensions = [11e-2,15e-2,8e-2] # measured by hand, and using technical drawing on interbotix docs website for reference
    gripper_offset =[-elem/2 for elem in gripper_dimensions] # y, z offsets are half of the arm size
    gripper_offset[0] = 2e-2 # Also measured by hand - the distance from the wrist to the beginning of the gripper
    # Determine corners
    gripper_min =[ gripper_origin [i]+ gripper_offset[i] for i in range(3)]
    gripper_max =[ gripper_min [i]+ gripper_dimensions[i] for i in range(3)]
    # transform bounding boxes
    gripper_corners = _bb_from_endpoints(gripper_min, gripper_max)
    gripper_corners = _roll_cube(gripper_corners, gripper_roll,gripper_origin)
    gripper_corners = _pitch_cube(gripper_corners, -wrist_angle, gripper_origin)
    gripper_corners = _roll_cube(gripper_corners, forearm_roll, gripper_origin)
    gripper_corners = _pitch_cube(gripper_corners, -elbow_angle, gripper_origin)
    gripper_corners = _pitch_cube(gripper_corners, -shoulder_angle, gripper_origin)
    gripper_corners = _yaw_cube(gripper_corners, waist_angle, gripper_origin)
    (gripper_min, gripper_max) = _endpoints_from_bb(gripper_corners)

    bounding_boxes["gripper"] = (gripper_min,gripper_max)
   
    "save positions"
    reader = Jsonreader("robot/assets/boundingboxes/")
    reader.clear("robot")
    reader.write("robot",bounding_boxes)
         
