import customtkinter as ctk
import time
from PIL import Image  # Make sure Pillow is installed
import os
# Set theme and appearance
ctk.set_appearance_mode("dark")           # Dark theme to match Prusa's style
ctk.set_default_color_theme("dark-blue")   # Consistent with modern design style

# Increase global widget scaling for tablet-friendly sizes
ctk.set_widget_scaling(2.0)

# Create main application window
root = ctk.CTk()
root.title("waffle UI")
root.geometry("1920x1080")  # change the size of the screen

# Configure grid layout for responsiveness in the root
root.grid_columnconfigure(1, weight=1)
root.grid_rowconfigure(0, weight=1)

# Sidebar menu (Left Navigation)
sidebar = ctk.CTkFrame(root, width=180, corner_radius=15, fg_color="gray17")
sidebar.grid(row=0, column=0, rowspan=5, sticky="nsew", padx=10, pady=10)

# Main content frame (right side, changes dynamically)
content_frame = ctk.CTkFrame(root, corner_radius=15, fg_color="gray13")
content_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
content_frame.grid_columnconfigure(0, weight=1)
content_frame.grid_rowconfigure(0, weight=1)

# Global variables for stats
total_waffles_made = 0
total_uptime = 0

def check_password_developer(entry_widget):
    if entry_widget.get() == "1234":
        show_developer_options()
    else:
        error_label = ctk.CTkLabel(content_frame, text="Wrong Code! Learn the way of the waffles", font=("Arial", 14), text_color="red")
        error_label.grid(row=1, column=0, sticky="ew", pady=5)

def show_page(page):
    # Clear current content
    for widget in content_frame.winfo_children():
        widget.destroy()
    content_frame.grid_columnconfigure(0, weight=1)
    content_frame.grid_rowconfigure(0, weight=1)
    
    if page == "Dashboard":
        dashboard_frame = ctk.CTkFrame(content_frame)
        dashboard_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        dashboard_frame.grid_columnconfigure(0, weight=1)
        dashboard_frame.grid_rowconfigure((0,1,2,3), weight=1)
        
        title = ctk.CTkLabel(dashboard_frame, text="Dashboard", font=("Arial", 32), text_color="white")
        title.grid(row=0, column=0, sticky="ew", pady=10)
        
        btn_start = ctk.CTkButton(
            dashboard_frame, 
            text="Start", 
            height=100, 
            font=("Arial", 26),
            command=start_progress, 
            fg_color="green", 
            hover_color="lightgreen"
        )
        btn_start.grid(row=1, column=0, sticky="ew", padx=20, pady=10)
        
        btn_stop = ctk.CTkButton(
            dashboard_frame, 
            text="Stop", 
            height=100, 
            font=("Arial", 26),
            command=stop_progress, 
            fg_color="red", 
            hover_color="lightcoral"
        )
        btn_stop.grid(row=2, column=0, sticky="ew", padx=20, pady=10)
        
        global progress_bar, estimated_time_label
        progress_bar = ctk.CTkProgressBar(dashboard_frame, width=400, mode="determinate")
        progress_bar.grid(row=3, column=0, sticky="ew", pady=10)
        estimated_time_label = ctk.CTkLabel(dashboard_frame, text="Estimated time to next waffle: 30s", font=("Arial", 20), text_color="white")
        estimated_time_label.grid(row=4, column=0, sticky="ew", pady=10)
        
    elif page == "Settings":
        settings_frame = ctk.CTkFrame(content_frame)
        settings_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        settings_frame.grid_columnconfigure(0, weight=1)
        settings_frame.grid_rowconfigure((0,1,2), weight=1)
        
        title = ctk.CTkLabel(settings_frame, text="Settings", font=("Arial", 28), text_color="white")
        title.grid(row=0, column=0, sticky="ew", pady=10)
        
        theme_label = ctk.CTkLabel(settings_frame, text="Choose Theme:", font=("Arial", 18), text_color="white")
        theme_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        theme_selector = ctk.CTkOptionMenu(settings_frame, values=["Dark", "Light", "System"], command=change_theme)
        theme_selector.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        
    elif page == "Stats":
        stats_frame = ctk.CTkFrame(content_frame)
        stats_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        stats_frame.grid_columnconfigure(0, weight=1)
        stats_frame.grid_rowconfigure((0,1), weight=1)
        
        title = ctk.CTkLabel(stats_frame, text="Stats", font=("Arial", 28), text_color="white")
        title.grid(row=0, column=0, sticky="ew", pady=10)
        stats_info = ctk.CTkLabel(
            stats_frame,
            text=f"Users Online: 128\nActive Sessions: 45\nTotal Waffles Made: {total_waffles_made}\nTotal Uptime: {total_uptime}s",
            font=("Arial", 20), text_color="white"
        )
        stats_info.grid(row=1, column=0, sticky="nsew", pady=10)
        
    elif page == "Developer":
        # Developer page container
        dev_frame = ctk.CTkFrame(content_frame)
        dev_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        dev_frame.grid_columnconfigure(0, weight=1)
        for r in range(5):
            dev_frame.grid_rowconfigure(r, weight=1)
        
        title = ctk.CTkLabel(dev_frame, text="Developer Mode", font=("Arial", 28), text_color="white")
        title.grid(row=0, column=0, columnspan=2, sticky="ew", pady=10)
        
        password_label = ctk.CTkLabel(dev_frame, text="Enter Code:", font=("Arial", 18), text_color="white")
        password_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        
        password_entry = ctk.CTkEntry(dev_frame, show="*")
        password_entry.grid(row=1, column=1, sticky="ew", padx=10, pady=5)
        
        # Container frame for keypad and image (2 columns)
        keypad_image_frame = ctk.CTkFrame(dev_frame)
        keypad_image_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        keypad_image_frame.grid_columnconfigure(0, weight=1)  # keypad column
        keypad_image_frame.grid_columnconfigure(1, weight=1)  # image column
        
        # Keypad frame (smaller cells now)
        keypad_frame = ctk.CTkFrame(keypad_image_frame)
        keypad_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        for row in range(4):
            keypad_frame.grid_rowconfigure(row, weight=1, minsize=60)
        for col in range(3):
            keypad_frame.grid_columnconfigure(col, weight=1, minsize=60)
        
        def update_entry(value):
            current_text = password_entry.get()
            password_entry.delete(0, 'end')
            password_entry.insert('end', current_text + value)
        
        def clear_entry():
            password_entry.delete(0, 'end')
        
        buttons = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
            ('Clear', 3, 0), ('0', 3, 1), ('Enter', 3, 2)
        ]
        
        for text, row, col in buttons:
            if text == "Clear":
                btn = ctk.CTkButton(keypad_frame, text=text, fg_color="red", command=clear_entry, height=60, font=("Arial", 18))
            elif text == "Enter":
                btn = ctk.CTkButton(keypad_frame, text=text, fg_color="green", command=lambda: check_password_developer(password_entry), height=60, font=("Arial", 18))
            else:
                btn = ctk.CTkButton(keypad_frame, text=text, command=lambda t=text: update_entry(t), height=60, font=("Arial", 18))
            btn.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
        

        try:
            # Expand the tilde to the full home directory path
            image_path = os.path.expanduser("~/git/vaffelgutta/skjerm/hacker.png")
            pil_image = Image.open(image_path)
            # Create a CTkImage with desired dimensions (adjust as needed)
            ctk_image = ctk.CTkImage(light_image=pil_image, dark_image=pil_image, size=(150,150))
            image_label = ctk.CTkLabel(keypad_image_frame, image=ctk_image, text="")
            image_label.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        except Exception as e:
            # If image loading fails, show a placeholder text
            image_label = ctk.CTkLabel(keypad_image_frame, text="Image\nNot Found", font=("Arial", 18), text_color="white")
            image_label.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

def show_developer_options():
    for widget in content_frame.winfo_children():
        widget.destroy()
    dev_options_frame = ctk.CTkFrame(content_frame)
    dev_options_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
    dev_options_frame.grid_columnconfigure(0, weight=1)
    for r in range(5):
        dev_options_frame.grid_rowconfigure(r, weight=1)
    
    title = ctk.CTkLabel(dev_options_frame, text="Developer Options", font=("Arial", 28), text_color="white")
    title.grid(row=0, column=0, sticky="ew", pady=10)
    
    btn_log = ctk.CTkButton(dev_options_frame, text="View Log", command=lambda: print("Viewing Log"), height=80, font=("Arial", 18))
    btn_log.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
    
    btn_vision = ctk.CTkButton(dev_options_frame, text="Machine Vision", command=lambda: print("Machine Vision Mode"), height=80, font=("Arial", 18))
    btn_vision.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
    
    btn_static = ctk.CTkButton(dev_options_frame, text="Static Mode", command=lambda: print("Static Mode"), height=80, font=("Arial", 18))
    btn_static.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
    
    btn_change_password = ctk.CTkButton(dev_options_frame, text="Change Password", command=lambda: print("Change Password"), height=80, font=("Arial", 18))
    btn_change_password.grid(row=4, column=0, sticky="ew", padx=10, pady=5)

def change_theme(new_mode: str):
    ctk.set_appearance_mode(new_mode.lower())

def start_progress():
    global total_waffles_made, total_uptime 
    estimated_time = 30  # seconds
    estimated_time_label.configure(text=f"Estimated time to next waffle: {estimated_time}s")
    progress_bar.configure(mode="determinate")
    
    def update_progress(remaining_time):
        nonlocal estimated_time
        if remaining_time > 0:
            progress_bar.set(1 - (remaining_time / estimated_time))
            remaining_time -= 1
            total_uptime += 1
            estimated_time_label.configure(text=f"Estimated time to next waffle: {remaining_time}s")
            root.after(1000, update_progress, remaining_time)
        else:
            progress_bar.set(0)
            estimated_time_label.configure(text="Waffle ready!")
            total_waffles_made += 1

    update_progress(estimated_time)

def stop_progress():
    progress_bar.configure(mode="indeterminate", value=0)
    estimated_time_label.configure(text="Estimated time to next waffle: 0s")

def emergency_stop_action():
    global emergency_stop_state
    if emergency_stop_state == "STOPPED":
        emergency_stop_state = "PRESSED"
        btn_emergency.configure(fg_color="red", text="Emergency Stop Pressed")
    elif emergency_stop_state == "PRESSED":
        emergency_stop_state = "HOMING"
        btn_emergency.configure(fg_color="red", text="Homing")
        root.after(2000, reset_emergency_stop)

def reset_emergency_stop():
    global emergency_stop_state
    emergency_stop_state = "STOPPED"
    btn_emergency.configure(fg_color="red", text="Emergency Stop")

# Initialize emergency stop state
emergency_stop_state = "STOPPED"

# Sidebar buttons with larger height and fonts for tablet use
btn_dashboard = ctk.CTkButton(sidebar, text="Dashboard", height=100, font=("Arial", 24), command=lambda: show_page("Dashboard"))
btn_dashboard.pack(pady=10)
btn_settings = ctk.CTkButton(sidebar, text="Settings", height=100, font=("Arial", 24), command=lambda: show_page("Settings"))
btn_settings.pack(pady=10)
btn_stats = ctk.CTkButton(sidebar, text="Stats", height=100, font=("Arial", 24), command=lambda: show_page("Stats"))
btn_stats.pack(pady=10)
btn_developer = ctk.CTkButton(sidebar, text="Developer", height=100, font=("Arial", 24), command=lambda: show_page("Developer"))
btn_developer.pack(pady=10)
btn_emergency = ctk.CTkButton(sidebar, text="Emergency Stop", height=100, font=("Arial", 24), fg_color="red", command=emergency_stop_action)
btn_emergency.pack(pady=15)
btn_exit = ctk.CTkButton(sidebar, text="Exit", height=100, font=("Arial", 24), fg_color="yellow", command=root.quit)
btn_exit.pack(pady=15)

# Start with Dashboard page
show_page("Dashboard")
root.mainloop()
