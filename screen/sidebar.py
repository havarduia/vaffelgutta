import customtkinter as ctk

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)

class SidebarButton(ctk.CTkButton):
    def __init__(self, master, name, command, **kwargs):
        super().__init__(master, text=name, font=FONT_BUTTON, command=command, **kwargs)

class Sidebar(ctk.CTkFrame):
    button_names = ["Emergency", "Home", "Stats", "Dev Mode"]

    def __init__(self, master, button_callback, **kwargs):
        super().__init__(master, **kwargs)
        self.button_callback = button_callback
        self.configure_grid()
        self.create_buttons()

    def configure_grid(self):
        self.grid_propagate(False)
        for i in range(6):
            self.grid_rowconfigure(i, weight=1 if 1 <= i <= 4 else 0)
        self.grid_columnconfigure(0, weight=1)

    def create_buttons(self):
        for i, name in enumerate(self.button_names):
            # Special styling for Emergency button
            if name == "Emergency":
                btn = SidebarButton(
                    self,
                    name=name,
                    command=lambda n=name: self.button_callback(n),
                    fg_color="#E53935",  # Red color
                    hover_color="#C62828",  # Darker red on hover
                    border_width=2,
                    border_color="#B71C1C"  # Even darker red border
                )
            else:
                btn = SidebarButton(self, name=name, command=lambda n=name: self.button_callback(n))

            btn.grid(row=i + 1, column=0, sticky="nsew", padx=3, pady=3)
