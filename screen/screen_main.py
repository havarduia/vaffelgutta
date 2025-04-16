import customtkinter as ctk
from screen.sidebar import Sidebar
from screen.pagecontroller import PageController
# Global appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)

class TouchScreenApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Touchscreen GUI")
        self.attributes("-fullscreen", True)

        self.grid_columnconfigure(0, minsize=300)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.sidebar = Sidebar(self, button_callback=self.on_button_click)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.page_controller = PageController(self)
        self.page_controller.grid(row=0, column=1, sticky="nsew")

    def on_button_click(self, button_name):
        if button_name == "Back":
            self.page_controller.go_back()
        else:
            self.page_controller.show_page(button_name)

if __name__ == "__main__":
    app = TouchScreenApp()
    app.mainloop()
