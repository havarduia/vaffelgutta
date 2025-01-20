import tkinter as tk
import subprocess

def open_script(script_name):
    try:
        subprocess.Popen(['python', script_name])  # Open the script in a new process
    except FileNotFoundError:
        print(f"Error: {script_name} not found.")

# Initialize the main window
root = tk.Tk()
root.geometry("1024x600")
root.title("Cam")
root.configure(bg="orange")

# Make the window full-screen for touch-friendly usability
root.attributes('-fullscreen', False)
root.resizable(False, False)

# Define button properties
button_width = 20
button_height = 5
font_style = ("Arial", 18)

# Create buttons
waffle_button = tk.Button(
    root, text="Home", command=root.destroy, 
    width=button_width, height=button_height, font=font_style, bg="yellow", fg="black"
)
waffle_button.place(x=10, y=10)

cam_button = tk.Button(
    root, text="Record pose", command=lambda: open_script(''), 
    width=button_width, height=button_height, font=font_style, bg="green", fg="white"
)
cam_button.place(x=1024 - 10 - 350, y=10)  # Right corner

debug_button = tk.Button(
    root, text="Ir cam", command=lambda: open_script(''), 
    width=button_width, height=button_height, font=font_style, bg="red", fg="white"
)
debug_button.place(x=10, y=600 - 10 - 200)  # Bottom left corner

games_button = tk.Button(
    root, text="RGB cam", command=lambda: open_script(''), 
    width=button_width, height=button_height, font=font_style, bg="blue", fg="white"
)
games_button.place(x=1024 - 10 - 350, y=600 - 10 - 200)  # Bottom right corner

# Run the main loop
root.mainloop()
