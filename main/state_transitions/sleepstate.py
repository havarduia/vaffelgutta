
from main.waffle_states.waffle_states import State
from camera.vision import Vision

def sleepstate(state: "CurrentState", bot: "Wafflebot"):

    try:
        bot.release()
        bot.go_to_home_pose()
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return

    state.set(State.HOME)

import traceback
import time

def start(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    print("Starting waffle making sequence from SLEEP state")

    try:
        # Step 1: Run vision system
        print("Running vision system...")
        try:
            vision.run_once()
            print("Vision system run completed successfully")
        except Exception as vision_error:
            print(f"ERROR running vision system: {str(vision_error)}")
            print(traceback.format_exc())
            # Continue even if vision fails
            print("Continuing despite vision error...")

        # Step 2: Release gripper
        print("Releasing gripper...")
        try:
            bot.release()
            print("Gripper released successfully")
        except Exception as gripper_error:
            print(f"ERROR releasing gripper: {str(gripper_error)}")
            print(traceback.format_exc())
            # Continue even if gripper fails
            print("Continuing despite gripper error...")

        # Step 3: Move to home position
        print("Moving to home position...")
        try:
            # Try a direct approach with joint positions if available
            if hasattr(bot.arm, 'set_joint_positions'):
                print("Using direct joint positions for home pose")
                # Set to a safe position - all zeros
                bot.arm.set_joint_positions([0, 0, 0, 0, 0, 0], moving_time=2.0)
                time.sleep(2)  # Wait for movement to complete
                print("Direct joint movement completed")
            else:
                # Fall back to standard method
                bot.go_to_home_pose()

            print("Robot successfully moved to home position")
        except Exception as home_error:
            print(f"ERROR moving to home position: {str(home_error)}")
            print(traceback.format_exc())
            print("Failed to move to home position, but continuing...")

    except FloatingPointError: # unused error used as signal.
        print("ERROR: Emergency stop signal received in SLEEP state")
        state.set(State.ERROR)
        return
    except Exception as e:
        print(f"ERROR in SLEEP state: {str(e)}")
        print(traceback.format_exc())
        state.set(State.ERROR)
        return

    print("Transitioning from SLEEP to HOME state")
    state.set(State.HOME)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.state_transitions.waffle_states.waffle_states import CurrentState
