import tkinter as tk
from tkinter import ttk
import subprocess
from datetime import timedelta

def open_script(script_name):
    try:
        subprocess.Popen(['python', script_name])  # Open the script in a new process
    except FileNotFoundError:
        print(f"Error: {script_name} not found.")

# Initialize the main window
root = tk.Tk()
root.geometry("1024x600")
root.title("Waffle Robot Monitor")
root.configure(bg="black")

# Make the window full-screen for touch-friendly usability
root.attributes('-fullscreen', False)
root.resizable(False, False)

# Define button properties
button_width = 20
button_height = 5
font_style = ("Arial", 18)

# Variables to hold data
waffles_made = 0
next_waffle_time = timedelta(seconds=30)  # Assuming 30 seconds for the next waffle

# Update function for the countdown
def update_timer():
    global next_waffle_time
    if next_waffle_time > timedelta(seconds=0):
        next_waffle_time -= timedelta(seconds=1)
        timer_label.config(text=f"Next waffle in: {next_waffle_time}")
        progress_var.set(30 - next_waffle_time.seconds)  # Update progress bar
    else:
        global waffles_made
        waffles_made += 1
        waffles_label.config(text=f"Waffles made: {waffles_made}")
        next_waffle_time = timedelta(seconds=30)  # Reset timer for next waffle
    root.after(1000, update_timer)

# Create buttons
home_button = tk.Button(
    root, text="Home", command=root.destroy, 
    width=button_width, height=button_height, font=font_style, bg="yellow", fg="black"
)
home_button.place(x=10, y=10)

waffles_label = tk.Label(root, text=f"Waffles made: {waffles_made}", font=("Arial", 28), bg="black", fg="orange")
waffles_label.place(x=1024 - 10 - 350, y=100)  # Right corner


Startbutton = tk.Button(
    root, text="Start", command=lambda: open_script('mimic.py'), 
    width=button_width, height=button_height, font=font_style, bg="green", fg="white"
)
Startbutton.place(x=10, y=600 - 160 - 200)  # Bottom left corner 

Stopbutton = tk.Button(
    root, text="Stop", command=lambda: open_script('mimic.py'), 
    width=button_width, height=button_height, font=font_style, bg="red", fg="white"
)
Stopbutton.place(x=10, y=600 - 10 - 200)  # Bottom left corner


# Add progress bar and labels
progress_var = tk.IntVar()
progress_bar = ttk.Progressbar(root, orient="horizontal", length=300, mode="determinate", variable=progress_var, maximum=30)
progress_bar.place(x=1024 - 10 - 350, y=600 - 10 - 200)  # Centered progress bar


timer_label = tk.Label(root, text=f"Next waffle in: {next_waffle_time}", font=("Arial", 25), bg="black", fg="orange")
timer_label.place(x=1024 - 25 - 350, y=600 - 80 - 200) # Above progress bar

# Start the countdown
update_timer()

# Run the main loop
root.mainloop()