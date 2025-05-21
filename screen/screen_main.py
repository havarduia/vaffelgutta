import customtkinter as ctk
from screen.sidebar import Sidebar
from screen.pagecontroller import PageController
from screen.notification_manager import NotificationManager
from robot.tools.maleman import MaleMan
# Global appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)

class TouchScreenApp(ctk.CTk):
    def __init__(self, maleman: MaleMan):
        super().__init__()
        self.title("Touchscreen GUI")
        self.attributes("-fullscreen", True)

        self.grid_columnconfigure(0, minsize=300)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create the notification manager (must be created before pages)
        self.notification_manager = NotificationManager(self)
        self.maleman = maleman

        self.sidebar = Sidebar(self, button_callback=self.on_button_click)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.page_controller = PageController(self)
        self.page_controller.grid(row=0, column=1, sticky="nsew")

    def rxmsg(self, operation, msg):
        match operation:
            case "manual_mode_collision":
                botbox, objbox = msg
                text = f"""
                Collision has been found along path.
                {botbox}
                has collided with 
                {objbox}.
                Do you want to move anyway?
                """
                self.show_confirmation(text,self.send_collision_feedback)
            case "show_message":
                self.show_notification(msg)
            case _:
                raise NotImplementedError
        
    def send_collision_feedback(self, result):
        self.maleman.send_male("robot", "collision_detected_response", result)

    def on_button_click(self, button_name):
        if button_name == "Back":
            self.page_controller.go_back()
        else:
            self.page_controller.show_page(button_name)

    def show_notification(self, message, popup_type="info", auto_hide=True, position="top"):
        """Show a notification popup.

        This method can be called from anywhere in the application.
        """
        self.notification_manager.show(message, popup_type, auto_hide, position)

    def show_marker_status(self, detected=True, marker_id=None, position="top"):
        """Show a notification about marker detection status.

        This method can be called from anywhere in the application.
        """
        self.notification_manager.show_marker_status(detected, marker_id, position)

    def show_confirmation(self, message, callback=None, position="center"):
        """Show a confirmation dialog with Yes/No buttons.

        Args:
            message: The message to display
            callback: A function to call with the result (True for Yes, False for No)
            position: Where to position the dialog ("top", "center", "bottom")

        This method can be called from anywhere in the application.
        """
        self.notification_manager.show_confirmation(message, callback, position)

if __name__ == "__main__":
    maleman = MaleMan()
    app = TouchScreenApp(maleman)
    app.mainloop()
