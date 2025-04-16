import tkinter as tk
from tkinter import font
from PIL import Image, ImageTk  # at the top

class TouchInterface(tk.Tk):
    def __init__(self,
                 width=1920,
                 height=1080,
                 sidebar_width=300,
                 button_font_size=20,
                 main_font_size=24,
                 button_padx=30,
                 button_pady=30,
                 # Waffle-themed colors:
                 sidebar_bg="#8B4513",       # Saddle Brown
                 main_area_bg="#F5DEB3",       # Wheat color
                 button_bg="#D2B48C",         # Tan color
                 button_font_color="black",   # Button text color
                 active_button_bg="#A0522D",  # Sienna for active state
                 main_text_color="black"):    # Main area text color
        super().__init__()
        
        # Store configuration parameters
        self.screen_history = []
        self.current_screen_content = None
        self.width = width
        self.height = height
        self.sidebar_width = sidebar_width
        self.button_font_size = button_font_size
        self.main_font_size = main_font_size
        self.button_padx = button_padx
        self.button_pady = button_pady
        self.sidebar_bg = sidebar_bg
        self.main_area_bg = main_area_bg
        self.button_bg = button_bg
        self.button_font_color = button_font_color
        self.active_button_bg = active_button_bg
        self.main_text_color = main_text_color

        self.title("Touch Screen Interface")
        self.geometry(f"{self.width}x{self.height}")
        self.screen_backgrounds = {
            "Home": ImageTk.PhotoImage(Image.open("/home/havard/git/vaffelgutta/screen/home_bg.png").resize((self.width - self.sidebar_width, self.height))),
            "Stats": ImageTk.PhotoImage(Image.open("/home/havard/git/vaffelgutta/screen/stats_bg.png").resize((self.width - self.sidebar_width, self.height))),
            "Dev Mode": ImageTk.PhotoImage(Image.open("/home/havard/git/vaffelgutta/screen/dev_bg.png").resize((self.width - self.sidebar_width, self.height))),
            "Emergency": ImageTk.PhotoImage(Image.open("/home/havard/git/vaffelgutta/screen/emergency_bg.png").resize((self.width - self.sidebar_width, self.height))),
            "Default": ImageTk.PhotoImage(Image.open("/home/havard/git/vaffelgutta/screen/default_bg.png").resize((self.width - self.sidebar_width, self.height)))
        }
        # Set a larger default font for touch friendliness
        default_font = font.nametofont("TkDefaultFont")
        default_font.configure(size=self.button_font_size)

        # Create a container for both sidebar and main area
        container = tk.Frame(self)
        container.pack(fill='both', expand=True)

        # Sidebar frame on the left
        self.sidebar = tk.Frame(container, bg=self.sidebar_bg, width=self.sidebar_width)
        self.sidebar.pack(side="left", fill="y")

        # Main area frame on the right
        self.main_area = tk.Frame(container, bg=self.main_area_bg)
        self.main_area.pack(side="right", fill="both", expand=True)

        # Add a label to the main area as a placeholder for content, with configurable font color
        self.main_label = tk.Label(
            self.main_area,
            text="Welcome",
            bg=self.main_area_bg,
            fg=self.main_text_color,
            font=("Helvetica", self.main_font_size)
        )
        self.main_label.pack(pady=20)

        # Create the sidebar buttons with the configurable styling
        self.create_emergency_button()
        self.create_sidebar_button("Home", self.go_home)
        self.create_sidebar_button("Stats", self.show_stats)
        self.create_sidebar_button("Dev Mode", self.dev_mode)
        self.create_sidebar_button("Back", self.go_back)

    def create_sidebar_button(self, text, command):
        """Creates and packs a touch-friendly sidebar button with configurable styling."""
        btn = tk.Button(
            self.sidebar,
            text=text,
            command=command,
            font=("Helvetica", self.button_font_size),
            bg=self.button_bg,
            fg=self.button_font_color,
            activebackground=self.active_button_bg,
            relief="flat",
            bd=0,
            padx=self.button_padx,
            pady=self.button_pady
        )
        btn.pack(fill="x", pady=5, padx=10)
        return btn
    
    def create_emergency_button(self):
        """Creates a red circular emergency button inside the sidebar."""
        canvas_size = 200
        padding = 2  # Space between the circle and canvas edge

        self.emergency_canvas = tk.Canvas(
            self.sidebar,
            width=canvas_size,
            height=canvas_size,
            bg=self.sidebar_bg,
            highlightthickness=0
        )
        self.emergency_canvas.pack(pady=2)

        # Draw red circle fully inside the canvas
        self.emergency_circle = self.emergency_canvas.create_oval(
            padding, padding,
            canvas_size - padding, canvas_size - padding,
            fill="red", outline=""
        )

        # Add centered white "!" text
        self.emergency_text = self.emergency_canvas.create_text(
            canvas_size // 2, canvas_size // 2,
            text="!", fill="white",
            font=("Helvetica", int(self.button_font_size * 1.5), "bold")
        )

        # Bind both circle and text to the emergency function
        for tag in (self.emergency_circle, self.emergency_text):
            self.emergency_canvas.tag_bind(tag, "<Button-1>", self.emergency)

            
    def flash_emergency_button(self):
        """Toggle the emergency button color to create a flashing effect."""
        if getattr(self, "stop_flashing", False):
            return  # Exit if flashing should stop

        # Alternate between red and dark red
        new_color = "darkred" if self.flash_state else "red"
        self.emergency_canvas.itemconfig(self.emergency_circle, fill=new_color)
        self.flash_state = not self.flash_state

        # Continue flashing every 500ms
        self.after(500, self.flash_emergency_button)

    def set_main_background(self, screen_name):
        """Set the background image for the given screen."""
        bg_image = self.screen_backgrounds.get(screen_name, self.screen_backgrounds["Default"])
        
        # Create label to hold background image
        self.bg_label = tk.Label(self.main_area, image=bg_image)
        self.bg_label.image = bg_image  # prevent garbage collection
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        
    def update_main_area(self, message, screen_name=None):
        """Update the main display with the given message and track history."""
        if screen_name is None:
            screen_name = message  # fallback if not explicitly passed

        if self.current_screen_content is not None:
            self.screen_history.append(self.current_screen_content)

        self.current_screen_content = screen_name

        # Clear existing widgets
        for widget in self.main_area.winfo_children():
            widget.destroy()

        # Set background
        self.set_main_background(screen_name)
        
        # Display the message
        self.main_label = tk.Label(
            self.main_area,
            text=message,
            bg=self.main_area_bg,
            fg=self.main_text_color,
            font=("Helvetica", self.main_font_size)
        )
        self.main_label.pack(pady=20)

    # Callback functions for sidebar buttons
    def go_home(self):
        self.update_main_area("Home Screen", screen_name="Home")

    def show_stats(self):
        self.update_main_area("Stats Screen", screen_name="Stats")


    def dev_mode(self):
        """Display an integrated keypad in the main area to enter a 4-digit code."""
        # Clear the main area to insert the keypad
        for widget in self.main_area.winfo_children():
            widget.destroy()

        # Create a keypad frame within the main area
        self.keypad_frame = tk.Frame(self.main_area, bg=self.main_area_bg)
        self.keypad_frame.pack(expand=True)

        # Initialize the entered code variable
        self.entered_code = ""

        # Label for instructions
        instruction_label = tk.Label(
            self.keypad_frame,
            text="Enter 4-digit code:",
            font=("Helvetica", self.main_font_size),
            bg=self.main_area_bg,
            fg=self.main_text_color
        )
        instruction_label.grid(row=0, column=0, columnspan=3, pady=10)

        # Label to display asterisks for entered digits
        self.display_label = tk.Label(
            self.keypad_frame,
            text="",
            font=("Helvetica", self.main_font_size),
            bg=self.main_area_bg,
            fg=self.main_text_color
        )
        self.display_label.grid(row=1, column=0, columnspan=3, pady=10)

        # Define layout for digit buttons and function buttons
        buttons = [
            ('1', 2, 0), ('2', 2, 1), ('3', 2, 2),
            ('4', 3, 0), ('5', 3, 1), ('6', 3, 2),
            ('7', 4, 0), ('8', 4, 1), ('9', 4, 2),
            ('Clear', 5, 0), ('0', 5, 1), ('Enter', 5, 2)
        ]
        # Create buttons in the keypad frame
        for (text, row, col) in buttons:
            if text == "Clear":
                action = lambda: self.clear_code()
            elif text == "Enter":
                action = lambda: self.check_dev_code()
            else:
                action = lambda digit=text: self.append_digit(digit)
            btn = tk.Button(
                self.keypad_frame,
                text=text,
                font=("Helvetica", self.button_font_size),
                width=5, height=2,
                command=action
            )
            btn.grid(row=row, column=col, padx=5, pady=5)

    def append_digit(self, digit):
        """Appends a digit to the entered code and updates the display."""
        if len(self.entered_code) < 5:
            self.entered_code += str(digit)
            self.display_label.config(text="*" * len(self.entered_code))
        if len(self.entered_code) == 5:
            self.check_dev_code()

    def clear_code(self):
        """Clears the entered code."""
        self.entered_code = ""
        self.display_label.config(text="")

    def check_dev_code(self):
        """Checks if the entered 4-digit code is correct."""
        correct_code = "42458"  # Change this to your desired dev code
        if self.entered_code == correct_code:
            # Remove keypad and activate Dev Mode
            self.keypad_frame.destroy()
            self.update_main_area("Dev Mode Activated", screen_name="Dev Mode")
            self.entered_code = ""
        else:
            # Show error message briefly before resetting keypad
            self.display_label.config(text="IHagle", fg="red")
            self.after(1000, self.reset_keypad)

    def reset_keypad(self):
        """Resets the keypad after an incorrect attempt."""
        self.entered_code = ""
        self.display_label.config(text="", fg=self.main_text_color)

    def emergency(self, event=None):
        """Toggle Emergency mode ON/OFF and start/stop flashing."""
        if getattr(self, "emergency_active", False):
            # Deactivate emergency mode
            self.emergency_active = False
            self.update_main_area("Emergency Cleared")
            self.stop_flashing = True
            self.emergency_canvas.itemconfig(self.emergency_circle, fill="red")
        else:
            # Activate emergency mode
            self.emergency_active = True
            self.stop_flashing = False
            self.update_main_area("🚨 EMERGENCY MODE 🚨")
            self.flash_state = True
            self.flash_emergency_button()


    def go_back(self):
        """Go back to the previous screen, asking for password again if Dev Mode."""
        if self.screen_history:
            last_screen = self.screen_history.pop()
            if last_screen == "Dev Mode":
                self.dev_mode()  # re-trigger keypad
            else:
                self.update_main_area(last_screen, screen_name=last_screen)

def screen():
    app = TouchInterface(
        width=1920,
        height=1080,
        sidebar_width=300,
        button_font_size=20,
        main_font_size=24,
        button_padx=30,
        button_pady=70,
        sidebar_bg="#8B4513",
        main_area_bg="#F5DEB3",
        button_bg="#D2B48C",
        button_font_color="black",
        active_button_bg="#A0522D",
        main_text_color="black"
    )
    app.mainloop()

if __name__ == "__main__":
    screen()
