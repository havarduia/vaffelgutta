import customtkinter as ctk

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class Sidebar(ctk.CTkFrame):
    def __init__(self, master, button_callback, **kwargs):
        super().__init__(master, **kwargs)
        self.button_callback = button_callback
        self.configure(width=300)

        # Grid configuration for spacing
        self.grid_rowconfigure(0, weight=0)  # Top padding
        for i in range(1, 6):
            self.grid_rowconfigure(i, weight=1)  # One row per button
        self.grid_rowconfigure(6, weight=0)  # Bottom padding
        self.grid_columnconfigure(0, weight=1)

        self.create_buttons()

    def create_buttons(self):
        button_names = ["Emergency", "Home", "Stats", "Dev Mode", "Back"]
        for i, name in enumerate(button_names):
            btn = ctk.CTkButton(
                self,
                text=name,
                font=("Arial", 36),
                command=lambda n=name: self.button_callback(n)
            )
            btn.grid(row=i + 1, column=0, sticky="nsew", padx=10, pady=10)

class BasePage(ctk.CTkFrame):
    def __init__(self, master, title, **kwargs):
        super().__init__(master, **kwargs)
        label = ctk.CTkLabel(self, text=title, font=("Arial", 64))
        label.pack(pady=100)

class EmergencyPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Emergency Page", **kwargs)


class HomePage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Home Page", **kwargs)


class StatsPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Stats Page", **kwargs)


class DevModePage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Developer Mode", **kwargs)

        self.correct_pin = "1234"
        self.pin_entered = False

        self.pin_frame = ctk.CTkFrame(self)
        self.content_frame = ctk.CTkFrame(self)

        self.pin_var = ""

        self.create_pin_entry()
        self.create_dev_content()

        self.show_pin_prompt()

    def create_pin_entry(self):
        label = ctk.CTkLabel(self.pin_frame, text="Enter PIN", font=("Arial", 48))
        label.pack(pady=20)

        self.pin_display = ctk.CTkEntry(self.pin_frame, font=("Arial", 48), justify="center", show="*",
                                        width=300, height=80)
        self.pin_display.pack(pady=20, ipady=10)

        keypad_frame = ctk.CTkFrame(self.pin_frame)
        keypad_frame.pack(pady=20)

        buttons = [
            ("1", 0, 0), ("2", 0, 1), ("3", 0, 2),
            ("4", 1, 0), ("5", 1, 1), ("6", 1, 2),
            ("7", 2, 0), ("8", 2, 1), ("9", 2, 2),
            ("C", 3, 0), ("0", 3, 1), ("OK", 3, 2),
        ]

        for (text, row, col) in buttons:
            btn = ctk.CTkButton(keypad_frame, text=text, width=100, height=100,
                                font=("Arial", 36), command=lambda t=text: self.keypad_press(t))
            btn.grid(row=row, column=col, padx=10, pady=10)

    def create_dev_content(self):
        label = ctk.CTkLabel(self.content_frame, text="Dev Mode Unlocked!", font=("Arial", 48))
        label.pack(pady=40)

        reboot_btn = ctk.CTkButton(self.content_frame, text="Reboot System", font=("Arial", 36), height=100, width=300)
        reboot_btn.pack(pady=20)

        logout_btn = ctk.CTkButton(self.content_frame, text="Lock", font=("Arial", 36), height=100, width=300,
                                command=self.reset)
        logout_btn.pack(pady=20)


    def keypad_press(self, key):
        if key == "C":
            self.pin_var = ""
        elif key == "OK":
            self.verify_pin()
            return
        else:
            if len(self.pin_var) < 10:
                self.pin_var += key

        self.pin_display.delete(0, ctk.END)
        self.pin_display.insert(0, self.pin_var)

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
        self.pin_frame.pack(expand=True)

    def show_dev_content(self):
        self.pin_frame.pack_forget()
        self.content_frame.pack(expand=True)

    def reset(self):
        self.pin_var = ""
        self.pin_entered = False
        self.pin_display.delete(0, ctk.END)
        self.show_pin_prompt()


class PageController(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.pages = {}
        self.current_page = None
        self.page_history = []
        self.create_pages()

    def create_pages(self):
        self.pages = {
            "Emergency": EmergencyPage(self),
            "Home": HomePage(self),
            "Stats": StatsPage(self),
            "Dev Mode": DevModePage(self),
        }

        for page in self.pages.values():
            page.place(relx=0, rely=0, relwidth=1, relheight=1)

        self.show_page("Home", remember=False)

    def show_page(self, page_name, remember=True):
        if page_name not in self.pages:
            return

        if self.current_page and remember:
            # Save current page to history stack
            self.page_history.append(self.current_page)

        if self.current_page:
            self.current_page.lower()

        self.current_page = self.pages[page_name]
        self.current_page.lift()

    def go_back(self):
        if self.page_history:
            previous_page = self.page_history.pop()
            if self.current_page:
                self.current_page.lower()
            self.current_page = previous_page

            # Reset DevMode if we're going back to it
            if isinstance(self.current_page, DevModePage):
                self.current_page.reset()

            self.current_page.lift()


class TouchScreenApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Touchscreen GUI")
        
        # Make the window fullscreen
        self.attributes("-fullscreen", True)

        self.sidebar = Sidebar(self, button_callback=self.on_button_click)
        self.sidebar.pack(side="left", fill="y")

        self.page_controller = PageController(self)
        self.page_controller.pack(side="right", expand=True, fill="both")

    def on_button_click(self, button_name):
        if button_name == "Back":
            self.page_controller.go_back()
        else:
            self.page_controller.show_page(button_name)


if __name__ == "__main__":
    app = TouchScreenApp()
    app.mainloop()
