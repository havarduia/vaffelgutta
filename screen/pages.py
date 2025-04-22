import os
from PIL import Image
import customtkinter as ctk

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
        ctk.CTkButton(self, text="Back", font=FONT_BUTTON, command=self.go_back).place(relx=1.0, rely=0.0, anchor="ne")

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

        # Add demo buttons to show the notification popups
        self.error_button = ctk.CTkButton(
            self,
            text="Show Marker Not Detected",
            font=("Arial", 20),
            command=self.show_marker_not_detected,
            width=300,
            height=50
        )
        self.error_button.place(relx=0.5, rely=0.4, anchor="center")

        self.success_button = ctk.CTkButton(
            self,
            text="Show Marker Detected",
            font=("Arial", 20),
            command=self.show_marker_detected,
            width=300,
            height=50
        )
        self.success_button.place(relx=0.5, rely=0.5, anchor="center")

        # Add a button to show a custom notification
        self.custom_button = ctk.CTkButton(
            self,
            text="Show Custom Notification",
            font=("Arial", 20),
            command=self.show_custom_notification,
            width=300,
            height=50
        )
        self.custom_button.place(relx=0.5, rely=0.6, anchor="center")

        self.hagle_button = ctk.CTkButton(
            self,
            text="HAGLE",
            font=("Arial", 20),
            command=self.shout_hagle,
            width=300,
            height=50
        )
        self.hagle_button.place(relx=0.5, rely=0.7, anchor="center")

        # Add a button to show a confirmation dialog
        self.confirm_button = ctk.CTkButton(
            self,
            text="Show Confirmation Dialog",
            font=("Arial", 20),
            command=self.show_confirmation_demo,
            width=300,
            height=50
        )
        self.confirm_button.place(relx=0.5, rely=0.8, anchor="center")

    def show_marker_not_detected(self):
            """Show a notification that a marker was not detected."""
            self.show_marker_status(detected=False)

    def show_marker_detected(self):
        """Show a notification that a marker was detected."""
        self.show_marker_status(detected=True, marker_id=42)

    def show_custom_notification(self):
        """Show a custom notification."""
        self.show_notification(
            message="This is a custom notification that can be shown from any page.",
            popup_type="info",
            auto_hide=True,
            position="center"
        )

    def shout_hagle(self):
        """Show a custom notification."""
        self.show_notification(
            message="HAGLE",
            popup_type="info",
            auto_hide=True,
            position="center"
        )

    def show_confirmation_demo(self):
        """Show a confirmation dialog demo."""
        self.show_confirmation(
            message="Vil du skyte deg selv?",
            callback=self.handle_confirmation_result
        )

    def handle_confirmation_result(self, result):
        """Handle the result of the confirmation dialog.

        Args:
            result: True if the user clicked Yes, False if the user clicked No
        """
        if result:
            self.show_notification(
                message="Pang!",
                popup_type="success",
                auto_hide=True
            )
        else:
            self.show_notification(
                message="En annen gang!",
                popup_type="error",
                auto_hide=True
            )


class EmergencyPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Emergency Page", background_image_filename="background.jpg", **kwargs)

        ctk.CTkButton(self, text="Emergency Stop", font=FONT_BUTTON, height=100, width=300).pack(pady=20)

class StatsPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Stats Page", background_image_filename="background.jpg", **kwargs)

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
        ctk.CTkLabel(self.content_frame, text="Dev Mode Unlocked!", font=FONT_TITLE).pack(pady=40)

        ctk.CTkButton(self.content_frame, text="Reboot System", font=FONT_BUTTON, height=100, width=300).pack(pady=20)
        ctk.CTkButton(self.content_frame, text="Lock", font=FONT_BUTTON, height=100, width=300,
                      command=self.reset).pack(pady=20)

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
