import customtkinter as ctk
import time

# Set theme and appearance
ctk.set_appearance_mode("dark")  # Dark theme to match Prusa's style
ctk.set_default_color_theme("dark-blue")  # Consistent with modern design style

# Create main application window
root = ctk.CTk()
root.title("waffle UI")
root.geometry("700x450") #change the size of the screen

# Configure grid layout for responsiveness
root.grid_columnconfigure(1, weight=1) #how the layout looks based on how many columns you want
root.grid_rowconfigure(0, weight=1)

# Sidebar menu (Left Navigation)
sidebar = ctk.CTkFrame(root, width=180, corner_radius=15, fg_color="gray17") #formating the looks of the navigation menu
sidebar.grid(row=0, column=0, rowspan=5, sticky="nswe", padx=10, pady=10)

# Main content frame (right side, changes dynamically)
content_frame = ctk.CTkFrame(root, corner_radius=15, fg_color="gray13")#configurating the right menu
content_frame.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)

# Global variables for stats
total_waffles_made = 0
total_uptime = 0

# Function to switch between pages
def show_page(page):
    # Clear current content
    for widget in content_frame.winfo_children():
        widget.destroy()
    
    if page == "Dashboard": #if dashboard is shown
        title = ctk.CTkLabel(content_frame, text="Dashboard", font=("Arial", 28), text_color="white")
        title.pack(pady=20)

        # Start and Stop buttons
        btn_start = ctk.CTkButton(content_frame, text="Start", height=60, font=("Arial", 18), command=start_progress, fg_color="green", hover_color="lightgreen")
        btn_start.pack(pady=10, padx=20, fill="x")

        btn_stop = ctk.CTkButton(content_frame, text="Stop", height=60, font=("Arial", 18), command=stop_progress, fg_color="red", hover_color="lightcoral")
        btn_stop.pack(pady=10, padx=20, fill="x")

        # Progress Bar
        global progress_bar
        progress_bar = ctk.CTkProgressBar(content_frame, width=300, mode="indeterminate")
        progress_bar.pack(pady=10)
        
        # Label for estimated time
        global estimated_time_label
        estimated_time_label = ctk.CTkLabel(content_frame, text="Estimated time to next waffle: 0s", font=("Arial", 14), text_color="white")
        estimated_time_label.pack(pady=10)

    elif page == "Settings":
        title = ctk.CTkLabel(content_frame, text="Settings", font=("Arial", 24), text_color="white")
        title.pack(pady=20)
        
        theme_label = ctk.CTkLabel(content_frame, text="Choose Theme:", font=("Arial", 16), text_color="white")
        theme_label.pack(pady=10)
        
        # Theme Selector
        theme_selector = ctk.CTkOptionMenu(content_frame, values=["Dark", "Light", "System"], command=change_theme)
        theme_selector.pack(pady=10)

    elif page == "Stats":
        title = ctk.CTkLabel(content_frame, text="Stats", font=("Arial", 24), text_color="white")
        title.pack(pady=20)

        global total_waffles_made, total_uptime
        stats_info = ctk.CTkLabel(content_frame, text=f"Users Online: 128\nActive Sessions: 45\nTotal Waffles Made: {total_waffles_made}\nTotal Uptime: {total_uptime}s", font=("Arial", 16), text_color="white")
        stats_info.pack(pady=10)

    elif page == "Developer":
        title = ctk.CTkLabel(content_frame, text="Developer Mode", font=("Arial", 24), text_color="white")
        title.pack(pady=20)

        password_label = ctk.CTkLabel(content_frame, text="Enter Code:", font=("Arial", 16), text_color="white")
        password_label.pack(pady=10)
        
        password_entry = ctk.CTkEntry(content_frame, show="*")
        password_entry.pack(pady=5)

        keypad_btn = ctk.CTkButton(content_frame, text="Open Keypad", command=lambda: open_keypad(password_entry))
        keypad_btn.pack(pady=10)

        def check_password():
            if password_entry.get() == "1234": #if you want to change the password
                show_developer_options()
            else:
                error_label = ctk.CTkLabel(content_frame, text="Wrong Code! learn the way of the waffles", font=("Arial", 12), text_color="red")
                error_label.pack()

        enter_button = ctk.CTkButton(content_frame, text="Enter", command=check_password) #button to enter in password
        enter_button.pack(pady=10) 

def open_keypad(entry_widget): #widget for the keypad
    """Opens a keypad to enter numbers"""
    keypad_window = ctk.CTkToplevel()
    keypad_window.title("Keypad")
    keypad_window.geometry("250x300")
    
    keypad_window.grid_columnconfigure((0, 1, 2), weight=1)#formating
    
    # Function to update entry field
    def update_entry(value):#enter funttion
        current_text = entry_widget.get()
        entry_widget.delete(0, 'end')
        entry_widget.insert('end', current_text + value)

    def clear_entry():#clear function
        entry_widget.delete(0, 'end')

    # Create buttons for numbers 0-9
    buttons = [
        ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
        ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
        ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ('0', 3, 1)
    ]

    for text, row, col in buttons:
        btn = ctk.CTkButton(keypad_window, text=text, width=60, height=60, command=lambda t=text: update_entry(t))
        btn.grid(row=row, column=col, padx=5, pady=5)

    # Add Clear and Enter buttons
    btn_clear = ctk.CTkButton(keypad_window, text="Clear", fg_color="red", width=60, height=60, command=clear_entry)
    btn_clear.grid(row=3, column=0, padx=5, pady=5)

    btn_enter = ctk.CTkButton(keypad_window, text="Enter", fg_color="green", width=60, height=60, command=keypad_window.destroy)
    btn_enter.grid(row=3, column=2, padx=5, pady=5)

def show_developer_options(): #choises within the developer menu
    for widget in content_frame.winfo_children():
        widget.destroy()

    title = ctk.CTkLabel(content_frame, text="Developer Options", font=("Arial", 24), text_color="white")
    title.pack(pady=20)

    btn_log = ctk.CTkButton(content_frame, text="View Log", command=lambda: print("Viewing Log"))
    btn_log.pack(pady=5)

    btn_vision = ctk.CTkButton(content_frame, text="Machine Vision", command=lambda: print("Machine Vision Mode"))
    btn_vision.pack(pady=5)

    btn_static = ctk.CTkButton(content_frame, text="Static Mode", command=lambda: print("Static Mode"))
    btn_static.pack(pady=5)

    btn_change_password = ctk.CTkButton(content_frame, text="Change Password", command=lambda: print("Change Password"))
    btn_change_password.pack(pady=5)

# Function to change the theme based on selection
def change_theme(mode):
    ctk.set_appearance_mode(mode.lower())

def start_progress():
    global total_waffles_made, total_uptime 

    estimated_time = 30  # seconds
    estimated_time_label.configure(text=f"Estimated time to next waffle: {estimated_time}s")
    
    # Initialize progress bar with no max or value set initially
    progress_bar.configure(mode="determinate")
    
    def update_progress(remaining_time):
        if remaining_time > 0:
            # Update the progress bar value
            progress_bar.set(1 - (remaining_time / estimated_time))
            remaining_time -= 1
            total_uptime += 1
            # Update the label with the remaining time
            estimated_time_label.configure(text=f"Estimated time to next waffle: {remaining_time}s")
            # Use root.after to call update_progress again after 1000ms (1 second)
            root.after(1000, update_progress, remaining_time)
        else:
            # Once the progress reaches 0, show that the waffle is ready
            progress_bar.set(0)
            estimated_time_label.configure(text="Waffle ready!")
            total_waffles_made += 1  # Increment the waffles made count

    # Start the progress update
    update_progress(estimated_time)

    global total_waffles_made, total_uptime

    estimated_time = 30  # seconds
    estimated_time_label.configure(text=f"Estimated time to next waffle: {estimated_time}s")
    progress_bar.configure(mode="determinate", maximum=estimated_time, value=0)

    def update_progress(remaining_time):
        if remaining_time > 0:
            # Update the progress bar value
            progress_bar.set(1 - (remaining_time / estimated_time))
            remaining_time -= 1
            total_uptime += 1
            # Update the label with the remaining time
            estimated_time_label.configure(text=f"Estimated time to next waffle: {remaining_time}s")
            # Use root.after to call update_progress again after 1000ms (1 second)
            root.after(1000, update_progress, remaining_time)
        else:
            # Once the progress reaches 0, show that the waffle is ready
            progress_bar.set(0)
            estimated_time_label.configure(text="Waffle ready!")
            total_waffles_made += 1  # Increment the waffles made count

    # Start the progress update
    update_progress(estimated_time)

# Stop Progress
def stop_progress():
    progress_bar.configure(mode="indeterminate", value=0)
    estimated_time_label.configure(text="Estimated time to next waffle: 0s")

# Emergency stop logic with delay
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

# Sidebar buttons
btn_dashboard = ctk.CTkButton(sidebar, text="Dashboard", height=50, command=lambda: show_page("Dashboard"))
btn_dashboard.pack(pady=5)

btn_settings = ctk.CTkButton(sidebar, text="Settings", height=50, command=lambda: show_page("Settings"))
btn_settings.pack(pady=5)

btn_stats = ctk.CTkButton(sidebar, text="Stats", height=50, command=lambda: show_page("Stats"))
btn_stats.pack(pady=5)

btn_developer = ctk.CTkButton(sidebar, text="Developer", height=50, command=lambda: show_page("Developer"))
btn_developer.pack(pady=5)

btn_emergency = ctk.CTkButton(sidebar, text="Emergency Stop", height=50, fg_color="red", command=emergency_stop_action)
btn_emergency.pack(pady=10)

btn_exit = ctk.CTkButton(sidebar, text="Exit", fg_color="yellow", height=50, command=root.quit)
btn_exit.pack(pady=10)

show_page("Dashboard")
root.mainloop()
