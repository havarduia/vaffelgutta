"""
Waffle Making Robot Main Control Script

This script integrates the robot control system with a touchscreen GUI interface.
"""

import threading
import queue
import customtkinter as ctk
from camera.vision import Vision
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from main.waffle_states.waffle_states import State, CurrentState
from main.state_transitions import *
from screen.sidebar import Sidebar
from screen.pagecontroller import PageController

# Global appearance for the GUI
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)

# Global variables for communication between threads
state_queue = queue.Queue()
command_queue = queue.Queue()
status_queue = queue.Queue()
stop_event = threading.Event()

class WaffleApp(ctk.CTk):
    """Main application class that integrates the touchscreen GUI with the robot control system."""

    def __init__(self):
        super().__init__()
        self.title("Waffle Making Robot")
        self.attributes("-fullscreen", True)

        # Configure the grid layout
        self.grid_columnconfigure(0, minsize=300)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create the sidebar and page controller
        self.sidebar = Sidebar(self, button_callback=self.on_button_click)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.page_controller = PageController(self)
        self.page_controller.grid(row=0, column=1, sticky="nsew")

        # Initialize status display
        self.status_var = ctk.StringVar(value="Initializing...")
        self.status_label = ctk.CTkLabel(
            self,
            textvariable=self.status_var,
            font=("Arial", 16),
            fg_color="#333333",
            corner_radius=8
        )
        self.status_label.place(relx=0.5, rely=0.95, anchor="center", relwidth=0.9, relheight=0.05)

        # Start the status update thread
        self.status_update_thread = threading.Thread(target=self.update_status, daemon=True)
        self.status_update_thread.start()

    def on_button_click(self, button_name):
        """Handle sidebar button clicks."""
        if button_name == "Back":
            self.page_controller.go_back()
        elif button_name == "Emergency":
            # Put emergency stop command in the queue
            command_queue.put(("EMERGENCY_STOP", None))
            self.page_controller.show_page("Emergency")
        else:
            self.page_controller.show_page(button_name)

    def update_status(self):
        """Update the status display from the status queue."""
        while not stop_event.is_set():
            try:
                status = status_queue.get(timeout=0.1)
                self.status_var.set(status)
            except queue.Empty:
                pass

    def send_command(self, command, data=None):
        """Send a command to the robot control thread."""
        command_queue.put((command, data))

    def exit_application(self):
        """Clean exit of the application."""
        stop_event.set()
        self.quit()


def init_robot():
    """Initialize the robot and vision systems."""
    vision = Vision()
    state = CurrentState(State.SLEEP)  # Default to SLEEP state
    status_queue.put("Robot initialized in SLEEP state")
    return vision, state


def robot_control_thread():
    """Main robot control thread that processes commands and updates the robot state."""
    try:
        # Initialize robot and vision
        bot = Wafflebot(automatic_mode=True, detect_collisions=True)
        vision, state = init_robot()

        status_queue.put("Robot and vision systems initialized")

        # Main control loop
        while not stop_event.is_set():
            # Check for commands
            try:
                command, data = command_queue.get(block=False)

                # Process commands
                if command == "CHANGE_STATE":
                    try:
                        new_state = State[data]
                        state.set(new_state)
                        status_queue.put(f"State changed to {new_state.name}")
                    except (KeyError, ValueError):
                        status_queue.put(f"Invalid state: {data}")

                elif command == "EMERGENCY_STOP":
                    status_queue.put("EMERGENCY STOP ACTIVATED")
                    bot.safe_stop(slow=True)
                    # Don't exit the thread, just stop the robot

                elif command == "STOP_ROBOT":
                    status_queue.put("Robot stopped")
                    bot.safe_stop()
                    # Don't exit the thread, just stop the robot

                elif command == "MAKE_WAFFLE":
                    status_queue.put("Starting waffle making sequence")
                    # Set the state to start the waffle making sequence
                    state.set(State.HOME)

                elif command == "EXIT":
                    status_queue.put("Shutting down...")
                    bot.safe_stop()
                    break

            except queue.Empty:
                pass

            # Process current state
            current_state = state.get()
            state_queue.put(current_state)  # Update GUI with current state

            # Execute state action
            try:
                match current_state:
                    case State.REST:
                        status_queue.put("Robot is resting")
                        rest(state, bot)

                    case State.HOME:
                        status_queue.put("Moving to home position")
                        home(state, bot, vision)

                    case State.SLEEP:
                        status_queue.put("Robot is in sleep state")
                        sleepstate(state, bot)

                    case State.OPEN_IRON:
                        status_queue.put("Opening waffle iron")
                        open_iron(state, bot, vision)

                    case State.PICK_UP_SPRAY:
                        status_queue.put("Picking up spray")
                        pick_up_spray(state, bot, vision)

                    case State.SPRAY:
                        status_queue.put("Spraying waffle iron")
                        spray(state, bot)

                    case State.PUT_DOWN_SPRAY:
                        status_queue.put("Putting down spray")
                        put_down_spray(state, bot, vision)

                    case State.PICK_UP_LADLE:
                        status_queue.put("Picking up ladle")
                        pick_up_ladle(state, bot)

                    case State.CLOSE_IRON:
                        status_queue.put("Closing waffle iron")
                        close_iron(state, bot)

                    case State.FUN_TIME:
                        status_queue.put("Cooking waffle")
                        fun_time(state, bot)

                    case State.POUR_BATTER:
                        status_queue.put("Pouring batter")
                        pour_batter(state, bot)

                    case State.RETURN_LADLE:
                        status_queue.put("Returning ladle")
                        return_ladle(state, bot, vision)

                    case State.OPEN_IRON2:
                        status_queue.put("Opening iron to check waffle")
                        open_iron2(state, bot)

                    case State.RETURN_STICK:
                        status_queue.put("Returning stick")
                        return_stick(state, bot, vision)

                    case State.PICK_UP_WAFFLE:
                        status_queue.put("Picking up waffle")
                        pick_up_waffle(state, bot, vision)

                    case State.ERROR:
                        status_queue.put("ERROR: Robot needs attention!")
                        error(state, bot)

                    case _:
                        status_queue.put("Unknown state encountered")
                        bot.safe_stop(slow=True)

            except Exception as e:
                status_queue.put(f"Error in state {current_state.name}: {str(e)}")
                state.set(State.ERROR)

        # Clean shutdown
        bot.safe_stop()
        status_queue.put("Robot control thread terminated")

    except Exception as e:
        status_queue.put(f"Critical error: {str(e)}")
        handle_error(e)


def main():
    """Main function that starts the GUI and robot control threads."""
    # Start the robot control thread
    robot_thread = threading.Thread(target=robot_control_thread, daemon=True)
    robot_thread.start()

    # Create and start the GUI
    app = WaffleApp()

    # Update the HomePage to interact with the robot
    home_page = app.page_controller.pages["Home"]

    # Override button commands to interact with the robot
    home_page.on_make_waffle = lambda: app.send_command("MAKE_WAFFLE")
    home_page.change_state = lambda: app.page_controller.show_page("Dev Mode")  # Use Dev Mode for state changes
    home_page.stop_robot = lambda: app.send_command("STOP_ROBOT")
    home_page.exit = lambda: (app.send_command("EXIT"), app.after(1000, app.exit_application))

    # Connect the DevModePage to the robot control system
    dev_page = app.page_controller.pages["Dev Mode"]

    # Override the apply_state_change method to send commands to the robot
    dev_page.apply_state_change = lambda: app.send_command("CHANGE_STATE", dev_page.state_var.get())

    # Override the reboot_system method
    dev_page.reboot_system = lambda: (
        app.send_command("STOP_ROBOT"),
        status_queue.put("Rebooting system..."),
        app.after(2000, lambda: app.send_command("MAKE_WAFFLE"))
    )

    # Connect the EmergencyPage to the robot control system
    emergency_page = app.page_controller.pages["Emergency"]

    # Override the reset_robot method
    emergency_page.reset_robot = lambda: (
        app.send_command("CHANGE_STATE", "SLEEP"),
        status_queue.put("Resetting robot to SLEEP state..."),
        app.page_controller.show_page("Home")
    )

    # Override the exit_program method
    emergency_page.exit_program = lambda: (
        app.send_command("EXIT"),
        app.after(1000, app.exit_application)
    )

    # Connect the StatsPage to the robot control system
    stats_page = app.page_controller.pages["Stats"]

    # Initialize waffle counter
    waffle_counter = 0

    # Override the refresh_status method
    stats_page.refresh_status = lambda: (
        stats_page.update_state(state_queue.get().name if not state_queue.empty() else "UNKNOWN"),
        stats_page.update_status(app.status_var.get()),
        stats_page.update_counter(waffle_counter)
    )

    # Start a periodic update for the stats page (every 2 seconds)
    def update_stats():
        if app.page_controller.current_page == stats_page:
            try:
                current_state = state_queue.get(block=False)
                stats_page.update_state(current_state.name)

                # Update waffle counter when a waffle is completed
                nonlocal waffle_counter
                if current_state == State.PICK_UP_WAFFLE:
                    waffle_counter += 1
                    stats_page.update_counter(waffle_counter)

                stats_page.update_status(app.status_var.get())
            except queue.Empty:
                pass
        app.after(2000, update_stats)

    # Start the periodic update
    app.after(2000, update_stats)

    # Start the GUI main loop
    app.mainloop()

    # Set the stop event to terminate the robot thread
    stop_event.set()
    robot_thread.join(timeout=5.0)  # Wait for robot thread to terminate


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)
