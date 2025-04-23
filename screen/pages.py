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
    def __init__(self, master, title, background_image_filename=None, **kwargs):
        super().__init__(master, **kwargs)
        self.title = title

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


class HomePage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Home Page", background_image_filename="background.jpg", **kwargs)

class EmergencyPage(BasePage):
    def __init__(self, master, **kwargs):
        super().__init__(master, title="Emergency Page", background_image_filename="background.jpg", **kwargs)


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
