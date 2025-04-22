"""
Notification manager for displaying popup notifications across the application.
"""

import threading
import customtkinter as ctk

# Font constants
FONT_NOTIFICATION = ("Arial", 18)
FONT_CONFIRMATION = ("Arial", 18)
FONT_CONFIRMATION_BUTTON = ("Arial", 16)

class ConfirmationDialog(ctk.CTkFrame):
    """A Yes/No confirmation dialog that can appear on any page."""

    def __init__(self, master):
        """Initialize the confirmation dialog.

        Args:
            master: The root window (TouchScreenApp)
        """
        super().__init__(master)
        self.master = master
        self.visible = False
        self.callback = None

        # Configure the frame
        self.configure(corner_radius=10, fg_color="#333333")

        # Create the content
        self.message_label = ctk.CTkLabel(
            self,
            text="",
            font=FONT_CONFIRMATION,
            wraplength=400,
            text_color="#FFFFFF"
        )
        self.message_label.pack(padx=30, pady=(30, 20))

        # Create the buttons frame
        self.buttons_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.buttons_frame.pack(padx=20, pady=(0, 20), fill="x")

        # Create Yes and No buttons
        self.yes_button = ctk.CTkButton(
            self.buttons_frame,
            text="Yes",
            font=FONT_CONFIRMATION_BUTTON,
            command=self._on_yes,
            fg_color="#4CAF50",
            hover_color="#388E3C",
            width=120,
            height=40
        )
        self.yes_button.pack(side="left", padx=(0, 10))

        self.no_button = ctk.CTkButton(
            self.buttons_frame,
            text="No",
            font=FONT_CONFIRMATION_BUTTON,
            command=self._on_no,
            fg_color="#F44336",
            hover_color="#D32F2F",
            width=120,
            height=40
        )
        self.no_button.pack(side="right", padx=(10, 0))

        # Initially hide the dialog
        self.place_forget()

    def show(self, message, callback=None, position="center"):
        """Show the confirmation dialog with the specified message.

        Args:
            message: The message to display
            callback: A function to call with the result (True for Yes, False for No)
            position: Where to position the dialog ("top", "center", "bottom")
        """
        # Set the message
        self.message_label.configure(text=message)
        self.callback = callback

        # Position the dialog
        if position == "top":
            self.place(relx=0.5, rely=0.3, anchor="center")
        elif position == "center":
            self.place(relx=0.5, rely=0.5, anchor="center")
        elif position == "bottom":
            self.place(relx=0.5, rely=0.7, anchor="center")

        self.visible = True
        self.lift()  # Bring to front

    def hide(self):
        """Hide the confirmation dialog."""
        if self.visible:
            self.place_forget()
            self.visible = False

    def _on_yes(self):
        """Handle the Yes button click."""
        self.hide()
        if self.callback:
            self.callback(True)

    def _on_no(self):
        """Handle the No button click."""
        self.hide()
        if self.callback:
            self.callback(False)


class NotificationManager:
    """Manages popup notifications that can appear on any page."""

    # Define notification types with their colors
    TYPES = {
        "error": {"bg": "#FF5252", "fg": "#FFFFFF"},
        "warning": {"bg": "#FFC107", "fg": "#000000"},
        "info": {"bg": "#2196F3", "fg": "#FFFFFF"},
        "success": {"bg": "#4CAF50", "fg": "#FFFFFF"}
    }

    def __init__(self, master):
        """Initialize the notification manager.

        Args:
            master: The root window (TouchScreenApp)
        """
        self.master = master
        self.visible = False
        self.auto_hide_after = 3.0  # seconds
        self.auto_hide_timer = None

        # Create the notification frame
        self.frame = ctk.CTkFrame(master)
        self.frame.configure(corner_radius=10)

        # Create the content
        self.message_label = ctk.CTkLabel(
            self.frame,
            text="",
            font=FONT_NOTIFICATION,
            wraplength=400
        )
        self.message_label.pack(padx=20, pady=15)

        # Close button
        self.close_button = ctk.CTkButton(
            self.frame,
            text="×",
            width=20,
            height=20,
            command=self.hide,
            fg_color="transparent",
            hover_color="rgba(255, 255, 255, 0.2)",
            corner_radius=10
        )
        self.close_button.place(relx=1.0, rely=0.0, anchor="ne", x=-5, y=5)

        # Initially hide the popup
        self.frame.place_forget()

        # Create the confirmation dialog
        self.confirmation_dialog = ConfirmationDialog(master)

    def show(self, message, popup_type="info", auto_hide=True, position="top"):
        """Show the notification popup with the specified message and type.

        Args:
            message: The message to display
            popup_type: One of "error", "warning", "info", "success"
            auto_hide: Whether to automatically hide the popup after a delay
            position: Where to position the popup ("top", "center", "bottom")
        """
        # Cancel any existing auto-hide timer
        if self.auto_hide_timer:
            self.auto_hide_timer.cancel()
            self.auto_hide_timer = None

        # Set the message
        self.message_label.configure(text=message)

        # Apply the style for the notification type
        style = self.TYPES.get(popup_type, self.TYPES["info"])
        self.frame.configure(fg_color=style["bg"])
        self.message_label.configure(text_color=style["fg"])

        # Position the popup
        if position == "top":
            self.frame.place(relx=0.5, rely=0.1, anchor="center")
        elif position == "center":
            self.frame.place(relx=0.5, rely=0.5, anchor="center")
        elif position == "bottom":
            self.frame.place(relx=0.5, rely=0.9, anchor="center")

        self.visible = True
        self.frame.lift()  # Bring to front

        # Auto-hide if requested
        if auto_hide:
            self.auto_hide_timer = threading.Timer(self.auto_hide_after, self.hide)
            self.auto_hide_timer.daemon = True
            self.auto_hide_timer.start()

    def hide(self):
        """Hide the notification popup."""
        if self.visible:
            self.frame.place_forget()
            self.visible = False

            # Cancel any auto-hide timer
            if self.auto_hide_timer:
                self.auto_hide_timer.cancel()
                self.auto_hide_timer = None

    def show_marker_status(self, detected=True, marker_id=None, position="top"):
        """Show a notification about marker detection status.

        Args:
            detected: Whether the marker was detected
            marker_id: The ID of the marker (if detected)
            position: Where to position the notification
        """
        if detected:
            if marker_id is not None:
                message = f"Marker {marker_id} detected successfully."
            else:
                message = "Marker detected successfully."
            self.show(
                message=message,
                popup_type="success",
                auto_hide=True,
                position=position
            )
        else:
            self.show(
                message="Marker not detected. Please ensure the marker is visible to the camera.",
                popup_type="warning",
                auto_hide=False,  # Don't auto-hide error messages
                position=position
            )

    def show_confirmation(self, message, callback=None, position="center"):
        """Show a confirmation dialog with Yes/No buttons.

        Args:
            message: The message to display
            callback: A function to call with the result (True for Yes, False for No)
            position: Where to position the dialog ("top", "center", "bottom")
        """
        # Hide any existing notification
        self.hide()

        # Show the confirmation dialog
        self.confirmation_dialog.show(message, callback, position)
