import os
from PIL import Image
import customtkinter as ctk

from robot.tools.maleman import MaleMan

# Global appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)


class BasePage(ctk.CTkFrame):
    def __init__(self, master, title, background_image_filename=None, app=None, **kwargs):
        super().__init__(master, **kwargs)
        self.title = title
        self.app = app  # Store reference to the main app

        # Handle background image (do NOT pass it to super().__init__)
        if background_image_filename:
            image_path = os.path.join(os.path.expanduser("~/git/vaffelgutta/screen/images"), background_image_filename)
            if os.path.exists(image_path):
                self.bg_image = ctk.CTkImage(
                    light_image=Image.open(image_path),
                    dark_image=Image.open(image_path),
                    size=(self.winfo_width(), self.winfo_height())  # Set image size to the frame size
                )
                self.bg_label = ctk.CTkLabel(self, image=self.bg_image, text="")
                self.bg_label.place(relx=0, rely=0, relwidth=1, relheight=1)
            else:
                print(f"[Warning] Background image not found: {image_path}")

        self.create_title()
        self.create_back_button()

    def create_title(self):
        ctk.CTkLabel(self, text=self.title, font=FONT_TITLE).pack(pady=100)

    def create_back_button(self):
        # Create a larger back button with increased width, height, and padding for touch screens
        ctk.CTkButton(
            self,
            text="Back",
            font=FONT_BUTTON,
            command=self.go_back,
            width=200,  # Increased width
            height=100,  # Increased height
            corner_radius=15,  # Rounded corners
            hover_color="#1f538d",  # Darker blue on hover
            fg_color="#2a7fff",  # Brighter blue for visibility
            border_width=2,  # Add border for better visibility
            border_color="#1a4c8f"  # Darker border
        ).place(relx=0.98, rely=0.05, anchor="ne")  # Slightly adjusted position for better visibility

    def go_back(self):
        self.master.go_back()

    # Make sure the background image resizes with the frame
    def configure(self, **kwargs):
        super().configure(**kwargs)
        if hasattr(self, 'bg_label'):
            self.bg_label.place_forget()
            self.bg_image = ctk.CTkImage(
                light_image=Image.open(self.bg_image.light_image.filename),
                dark_image=Image.open(self.bg_image.dark_image.filename),
                size=(self.winfo_width(), self.winfo_height())
            )
            self.bg_label = ctk.CTkLabel(self, image=self.bg_image, text="")
            self.bg_label.place(relx=0, rely=0, relwidth=1, relheight=1)

    def show_notification(self, message, popup_type="info", auto_hide=True, position="top"):
        """Show a notification popup using the app's notification manager."""
        if self.app:
            self.app.show_notification(message, popup_type, auto_hide, position)

    def show_marker_status(self, detected=True, marker_id=None, position="top"):
        """Show a notification about marker detection status using the app's notification manager."""
        if self.app:
            self.app.show_marker_status(detected, marker_id, position)

    def show_confirmation(self, message, callback=None, position="center"):
        """Show a confirmation dialog with Yes/No buttons using the app's notification manager.

        Args:
            message: The message to display
            callback: A function to call with the result (True for Yes, False for No)
            position: Where to position the dialog ("top", "center", "bottom")
        """
        if self.app:
            self.app.show_confirmation(message, callback, position)


class HomePage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Home Page", background_image_filename="background.jpg", **kwargs)

        # Create a frame to hold the buttons
        self.button_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.button_frame.pack(expand=True, fill="both", padx=50, pady=(150, 50))

        # Configure grid for 2x2 button layout
        self.button_frame.grid_columnconfigure(0, weight=1)
        self.button_frame.grid_columnconfigure(1, weight=1)
        self.button_frame.grid_rowconfigure(0, weight=1)
        self.button_frame.grid_rowconfigure(1, weight=1)

        # Create 4 large touch-friendly buttons
        self.create_home_buttons()

    def create_home_buttons(self):
        """Create 4 large touch-friendly buttons in a 2x2 grid layout"""

        # Button configurations
        button_config = {
            "font": FONT_BUTTON,
            "width": 300,
            "height": 200,
            "corner_radius": 20,
            "border_width": 2,
        }

        # Create the 4 buttons
        self.make_waffle_btn = ctk.CTkButton(
            self.button_frame,
            text="Make Waffle",
            fg_color="#F9A825",  # Golden/orange color for waffles
            border_color="#F57F17",  # Darker orange border
            hover_color="#F57F17",  # Darker orange on hover
            command=lambda: self._on_make_waffle_click(),  # Use a lambda to call a private method
            **button_config
        )
        self.make_waffle_btn.grid(row=0, column=0, padx=30, pady=30, sticky="nsew")

        self.settings_btn = ctk.CTkButton(
            self.button_frame,
            text="Change State",
            fg_color="#2196F3",  # Blue for settings/state changes
            border_color="#1565C0",  # Darker blue border
            hover_color="#1565C0",  # Darker blue on hover
            command=lambda: self._on_change_state_click(),  # Use a lambda to call a private method
            **button_config
        )
        self.settings_btn.grid(row=0, column=1, padx=30, pady=30, sticky="nsew")

        self.status_btn = ctk.CTkButton(
            self.button_frame,
            text="Stop Robot",
            fg_color="#000000",  # Orange for caution/pause
            border_color="#E65100",  # Darker orange border
            hover_color="#E65100",  # Darker orange on hover
            command=lambda: self._on_stop_robot_click(),  # Use a lambda to call a private method
            **button_config
        )
        self.status_btn.grid(row=1, column=0, padx=30, pady=30, sticky="nsew")

        self.help_btn = ctk.CTkButton(
            self.button_frame,
            text="Exit",
            fg_color="#757575",  # Gray for exit
            border_color="#424242",  # Darker gray border
            hover_color="#424242",  # Darker gray on hover
            command=lambda: self._on_exit_click(),  # Use a lambda to call a private method
            **button_config
        )
        self.help_btn.grid(row=1, column=1, padx=30, pady=30, sticky="nsew")

    def _on_make_waffle_click(self):
        """Private method to handle the button click event"""
        print("Make Waffle button clicked - private handler")
        # Call the public method that will be overridden by the page connector
        self.on_make_waffle()

    def on_make_waffle(self):
        """Public method that will be overridden by the page connector"""
        print("Make Waffle button clicked - public handler")
        # This method will be overridden by the page connector

    def _on_change_state_click(self):
        """Private method to handle the Change State button click event"""
        print("Change State button clicked - private handler")
        # Call the public method that will be overridden by the page connector
        self.change_state()

    def change_state(self):
        """Public method that will be overridden by the page connector"""
        print("Change State button clicked - public handler")
        # This method will be overridden by the page connector

    def _on_stop_robot_click(self):
        """Private method to handle the Stop Robot button click event"""
        print("Stop Robot button clicked - private handler")
        # Call the public method that will be overridden by the page connector
        self.stop_robot()

    def stop_robot(self):
        """Public method that will be overridden by the page connector"""
        print("Stop Robot button clicked - public handler")
        # This method will be overridden by the page connector

    def _on_exit_click(self):
        """Private method to handle the Exit button click event"""
        print("Exit button clicked - private handler")
        # Call the public method that will be overridden by the page connector
        self.exit()

    def exit(self):
        """Public method that will be overridden by the page connector"""
        print("Exit button clicked - public handler")
        # This method will be overridden by the page connector
        self.master.master.quit()  # This will exit the application

class EmergencyPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="EMERGENCY STOP", background_image_filename="background.jpg", **kwargs)

        # Create a large warning message
        warning_frame = ctk.CTkFrame(self, fg_color="#B71C1C", corner_radius=20)
        warning_frame.pack(expand=True, fill="both", padx=100, pady=(150, 50))

        ctk.CTkLabel(
            warning_frame,
            text="EMERGENCY STOP ACTIVATED",
            font=("Arial", 60, "bold"),
            text_color="white"
        ).pack(pady=50)

        ctk.CTkLabel(
            warning_frame,
            text="Robot has been stopped for safety reasons.\nPlease check the robot before resuming operation.",
            font=("Arial", 24),
            text_color="white"
        ).pack(pady=20)

        # Add buttons for emergency actions
        button_frame = ctk.CTkFrame(warning_frame, fg_color="transparent")
        button_frame.pack(pady=50)

        # Reset button
        self.reset_btn = ctk.CTkButton(
            button_frame,
            text="Reset Robot",
            font=FONT_BUTTON,
            width=300,
            height=100,
            fg_color="#FF9800",
            hover_color="#E65100",
            command=self.reset_robot
        )
        self.reset_btn.pack(side="left", padx=20)

        # Exit button
        self.exit_btn = ctk.CTkButton(
            button_frame,
            text="Exit Program",
            font=FONT_BUTTON,
            width=300,
            height=100,
            fg_color="#757575",
            hover_color="#424242",
            command=self.exit_program
        )
        self.exit_btn.pack(side="left", padx=20)

    def reset_robot(self):
        """Reset the robot after emergency stop."""
        print("Resetting robot after emergency stop")
        # This will be connected to the robot control system in waffle_main.py

    def exit_program(self):
        """Exit the program."""
        print("Exiting program after emergency stop")
        # This will be connected to the robot control system in waffle_main.py

        ctk.CTkButton(self, text="Emergency Stop", font=FONT_BUTTON, height=100, width=300).pack(pady=20)

class StatsPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Robot Status", background_image_filename="background.jpg", **kwargs)

        # Create a frame for the stats
        self.stats_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.stats_frame.pack(expand=True, fill="both", padx=50, pady=(150, 50))

        # Current state display
        state_frame = ctk.CTkFrame(self.stats_frame, corner_radius=15)
        state_frame.pack(fill="x", pady=20, padx=50)

        ctk.CTkLabel(
            state_frame,
            text="Current Robot State:",
            font=("Arial", 24, "bold")
        ).pack(pady=(20, 10))

        self.state_var = ctk.StringVar(value="SLEEP")
        self.state_label = ctk.CTkLabel(
            state_frame,
            textvariable=self.state_var,
            font=("Arial", 36),
            text_color="#2196F3"
        )
        self.state_label.pack(pady=(0, 20))

        # Robot status display
        status_frame = ctk.CTkFrame(self.stats_frame, corner_radius=15)
        status_frame.pack(fill="x", pady=20, padx=50)

        ctk.CTkLabel(
            status_frame,
            text="Robot Status:",
            font=("Arial", 24, "bold")
        ).pack(pady=(20, 10))

        self.status_var = ctk.StringVar(value="Idle")
        self.status_label = ctk.CTkLabel(
            status_frame,
            textvariable=self.status_var,
            font=("Arial", 24),
            text_color="#4CAF50"
        )
        self.status_label.pack(pady=(0, 20))

        # Waffle counter
        counter_frame = ctk.CTkFrame(self.stats_frame, corner_radius=15)
        counter_frame.pack(fill="x", pady=20, padx=50)

        ctk.CTkLabel(
            counter_frame,
            text="Waffles Made:",
            font=("Arial", 24, "bold")
        ).pack(pady=(20, 10))

        self.counter_var = ctk.StringVar(value="0")
        self.counter_label = ctk.CTkLabel(
            counter_frame,
            textvariable=self.counter_var,
            font=("Arial", 36),
            text_color="#F9A825"
        )
        self.counter_label.pack(pady=(0, 20))

        # Refresh button
        self.refresh_btn = ctk.CTkButton(
            self.stats_frame,
            text="Refresh Status",
            font=FONT_BUTTON,
            width=300,
            height=80,
            fg_color="#2196F3",
            hover_color="#1565C0",
            command=self.refresh_status
        )
        self.refresh_btn.pack(pady=30)

    def refresh_status(self):
        """Refresh the robot status display."""
        print("Refreshing robot status")
        # This will be connected to the robot control system in waffle_main.py

    def update_state(self, state):
        """Update the displayed robot state."""
        self.state_var.set(state)

    def update_status(self, status):
        """Update the displayed robot status."""
        self.status_var.set(status)

    def update_counter(self, count):
        """Update the waffle counter."""
        self.counter_var.set(str(count))

class DevModePage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Developer Mode", background_image_filename="background.jpg", **kwargs)

        self.correct_pin = "1234"
        self.pin_var = ""
        self.pin_entered = False

        self.pin_frame = ctk.CTkFrame(self)
        self.content_frame = ctk.CTkFrame(self)

        self.pin_display = None
        self.keypad_frame = None

        self.create_pin_entry()
        self.create_dev_content()
        self.show_pin_prompt()

    def create_pin_entry(self):
        self.pin_frame.pack(expand=True, fill="both")

        ctk.CTkLabel(self.pin_frame, text="Enter PIN", font=FONT_TITLE).pack(pady=20)

        self.pin_display = ctk.CTkEntry(self.pin_frame, font=FONT_PIN_ENTRY, justify="center", show="*",
                                        width=300, height=80)
        self.pin_display.pack(pady=20, ipady=10)

        self.keypad_frame = ctk.CTkFrame(self.pin_frame)
        self.keypad_frame.pack(expand=True, fill="both", padx=40, pady=20)

        for i in range(4): self.keypad_frame.grid_rowconfigure(i, weight=1)
        for j in range(3): self.keypad_frame.grid_columnconfigure(j, weight=1)

        for text, row, col in [
            ("1", 0, 0), ("2", 0, 1), ("3", 0, 2),
            ("4", 1, 0), ("5", 1, 1), ("6", 1, 2),
            ("7", 2, 0), ("8", 2, 1), ("9", 2, 2),
            ("0", 3, 1)
        ]:
            ctk.CTkButton(
                self.keypad_frame, text=text, font=FONT_PIN_ENTRY,
                command=lambda t=text: self.keypad_press(t)
            ).grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

    def create_dev_content(self):
        ctk.CTkLabel(self.content_frame, text="Dev Mode Unlocked!", font=FONT_TITLE).pack(pady=20)

        # Create a frame for state selection
        state_frame = ctk.CTkFrame(self.content_frame)
        state_frame.pack(pady=20, fill="both", expand=True, padx=40)

        # Add a label for state selection
        ctk.CTkLabel(state_frame, text="Change Robot State", font=("Arial", 24)).pack(pady=10)

        # Create a dropdown for state selection
        self.state_var = ctk.StringVar(value="SLEEP")

        # Get state names from the State enum (will be imported in main/waffle_main.py)
        try:
            from main.waffle_states.waffle_states import State
            state_options = [state.name for state in State]
        except ImportError:
            # Fallback if we can't import the State enum
            state_options = ["SLEEP", "HOME", "REST", "OPEN_IRON", "ERROR"]

        state_dropdown = ctk.CTkOptionMenu(
            state_frame,
            values=state_options,
            variable=self.state_var,
            font=("Arial", 20),
            dropdown_font=("Arial", 20),
            width=300,
            height=50
        )
        state_dropdown.pack(pady=10)

        # Add a button to apply the state change
        self.apply_state_btn = ctk.CTkButton(
            state_frame,
            text="Apply State Change",
            font=FONT_BUTTON,
            height=80,
            width=300,
            fg_color="#2196F3",
            hover_color="#1565C0",
            command=self.apply_state_change
        )
        self.apply_state_btn.pack(pady=20)

        # Add buttons for system control
        ctk.CTkButton(
            self.content_frame,
            text="Reboot System",
            font=FONT_BUTTON,
            height=80,
            width=300,
            fg_color="#FF9800",
            hover_color="#E65100",
            command=self.reboot_system
        ).pack(pady=10)

        ctk.CTkButton(
            self.content_frame,
            text="Lock",
            font=FONT_BUTTON,
            height=80,
            width=300,
            fg_color="#757575",
            hover_color="#424242",
            command=self.reset
        ).pack(pady=10)

    def apply_state_change(self):
        """Apply the selected state change to the robot."""
        # This will be connected to the robot control system in waffle_main.py
        print(f"Changing state to: {self.state_var.get()}")
        # The actual implementation will be added in waffle_main.py

    def reboot_system(self):
        """Reboot the robot system."""
        # This will be connected to the robot control system in waffle_main.py
        print("Rebooting system...")
        # The actual implementation will be added in waffle_main.py

    def keypad_press(self, key):
        if len(self.pin_var) < len(self.correct_pin):
            self.pin_var += key
            self.pin_display.delete(0, ctk.END)
            self.pin_display.insert(0, self.pin_var)
            if len(self.pin_var) == len(self.correct_pin):
                self.verify_pin()

    def verify_pin(self):
        if self.pin_var == self.correct_pin:
            self.pin_entered = True
            self.show_dev_content()
        else:
            self.pin_var = ""
            self.pin_display.delete(0, ctk.END)
            self.pin_display.configure(placeholder_text="Wrong PIN", text_color="red")

    def show_pin_prompt(self):
        self.content_frame.pack_forget()
        self.pin_frame.pack(expand=True, fill="both")

    def show_dev_content(self):
        self.pin_frame.pack_forget()
        self.content_frame.pack(expand=True)

    def reset(self):
        self.pin_var = ""
        self.pin_entered = False

        if self.pin_display:
            self.pin_display.delete(0, ctk.END)
            self.pin_display.configure(placeholder_text="", text_color="white")

        self.show_pin_prompt()
