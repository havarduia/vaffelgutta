from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

def create_robot_state_from_joint_states(joints):
    joint_names =['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
    state_input = {}
    for key, value in zip(joint_names, joints):
        state_input[key] = value
    return create_robot_state(state_input)

def create_robot_state(state_input):
    # If robot state is a dict (user input),
    # Create a robot state and return it
    if isinstance(state_input, dict):
        js = JointState()        
        js.name = list(state_input.keys())
        js.position = list(state_input.values())

        return RobotState(joint_state = js)

    else:
        # If not a dict, use the current state.
        return RobotState()