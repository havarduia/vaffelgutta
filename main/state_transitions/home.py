from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.waffle_states.waffle_states import State, Tags
from camera.vision import Vision
import traceback
import time


def home(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    print("Executing HOME state")

    try:
        # Initialize actions and reader
        print("Initializing Actions and Jsonreader")
        actions = Actions(bot)
        reader = Jsonreader()

        # Clear camera readings
        print("Clearing camera readings")
        try:
            reader.clear("camera_readings")
            print("Camera readings cleared successfully")
        except Exception as clear_error:
            print(f"ERROR clearing camera readings: {str(clear_error)}")
            print(traceback.format_exc())
            print("Continuing despite error...")

        # Run vision if in automatic mode
        if bot.automatic_mode:
            print("Running vision in automatic mode")
            try:
                vision.run_once()
                print("Vision run completed successfully")
            except Exception as vision_error:
                print(f"ERROR running vision: {str(vision_error)}")
                print(traceback.format_exc())
                print("Continuing despite vision error...")

        # Read camera tags
        print("Reading camera tags")
        try:
            tags = reader.read("camera_readings")
            print(f"Camera tags read: {tags}")
        except Exception as read_error:
            print(f"ERROR reading camera tags: {str(read_error)}")
            print(traceback.format_exc())
            tags = {}  # Use empty dict if reading fails
            print("Using empty tags dictionary due to error")

        # Check for iron tag
        iron_tag_value = Tags.IRON_TAG.value
        print(f"Checking for iron tag (value: {iron_tag_value})")

        # Proceed if tag found or not in automatic mode
        if iron_tag_value in tags.keys() or int(iron_tag_value) in tags.keys() or not bot.automatic_mode:
            print("Iron tag found or not in automatic mode, proceeding with movements")

            try:
                # Execute movement sequence
                print("Moving to front_of_waffle_iron_a")
                bot.move("front_of_waffle_iron_a")
                time.sleep(0.5)

                print("Moving to front_of_waffle_iron_b")
                bot.move("front_of_waffle_iron_b")
                time.sleep(0.5)

                print("Moving to bottom_of_waffle_iron")
                bot.move("bottom_of_waffle_iron")
                time.sleep(0.5)

                print("Opening waffle iron")
                actions.open_waffle_iron()
                time.sleep(0.5)

                print("Moving to top_of_waffle_iron")
                bot.move("top_of_waffle_iron")
                time.sleep(0.5)

                print("Movement sequence completed successfully")

            except FloatingPointError:  # unused error used as signal.
                print("ERROR: Emergency stop signal received in HOME state")
                state.set(State.ERROR)
                return
            except Exception as move_error:
                print(f"ERROR during movement sequence: {str(move_error)}")
                print(traceback.format_exc())
                state.set(State.ERROR)
                return
        else:
            print("Iron tag not found, skipping movement sequence")

        # Transition to next state
        print("Transitioning from HOME to OPEN_IRON state")
        state.set(State.OPEN_IRON)

    except Exception as e:
        print(f"ERROR in HOME state: {str(e)}")
        print(traceback.format_exc())
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
