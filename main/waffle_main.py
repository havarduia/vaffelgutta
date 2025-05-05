"""
Waffle Making Robot Main Control Script

This script integrates the robot control system with a touchscreen GUI interface.
It uses an object-oriented approach to manage the robot and GUI components.
"""

import threading
import queue
import customtkinter as ctk
from robot.tools.errorhandling import handle_error
from screen.sidebar import Sidebar
from screen.pagecontroller import PageController
from main.robot_controller import Robot
from main.page_connector import Screen

# Global appearance for the GUI
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)


class WaffleApp(ctk.CTk):
    """Main application class that integrates the touchscreen GUI with the robot control system."""

    def __init__(self, command_queue, state_queue, status_queue, stop_event):
        """Initialize the application.

        Args:
            command_queue: Queue for sending commands to the robot controller
            state_queue: Queue for receiving state updates from the robot controller
            status_queue: Queue for receiving status messages from the robot controller
            stop_event: Event for signaling the application to stop
        """
        super().__init__()
        self.title("Waffle Making Robot")
        self.attributes("-fullscreen", True)

        # Store communication queues
        self.command_queue = command_queue
        self.state_queue = state_queue
        self.status_queue = status_queue
        self.stop_event = stop_event

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
            self.command_queue.put(("EMERGENCY_STOP", None))
            self.page_controller.show_page("Emergency")
        else:
            self.page_controller.show_page(button_name)

    def update_status(self):
        """Update the status display from the status queue."""
        while not self.stop_event.is_set():
            try:
                status = self.status_queue.get(timeout=0.1)
                self.status_var.set(status)
            except queue.Empty:
                pass

    def send_command(self, command, data=None):
        """Send a command to the robot control thread."""
        self.command_queue.put((command, data))

    def exit_application(self):
        """Clean exit of the application."""
        self.stop_event.set()
        self.quit()


def main():
    """Main function that starts the GUI and robot control threads."""
    # Create communication queues
    command_queue = queue.Queue()
    state_queue = queue.Queue()
    status_queue = queue.Queue()
    stop_event = threading.Event()

    # Create the robot controller
    robot_controller = Robot(command_queue, state_queue, status_queue, stop_event)

    # Start the robot control thread
    robot_thread = threading.Thread(target=robot_controller.run, daemon=True)
    robot_thread.start()

    # Create and start the GUI
    app = WaffleApp(command_queue, state_queue, status_queue, stop_event)

    # Connect the GUI pages to the robot controller
    page_connector = Screen(
        app,
        app.page_controller,
        app.send_command,
        state_queue,
        status_queue
    )

    # Start periodic updates for the stats page
    page_connector.start_stats_updates(robot_controller)

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
