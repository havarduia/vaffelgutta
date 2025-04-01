import customtkinter as ctk
import time

# Set theme and appearance
ctk.set_appearance_mode("dark")  # Dark theme to match Prusa's style
ctk.set_default_color_theme("dark-blue")  # Consistent with modern design style

# Create main application window
root = ctk.CTk()
root.title("Prusa-like UI")
root.geometry("700x450")

# Configure grid layout for responsiveness
root.grid_columnconfigure(1, weight=1)
root.grid_rowconfigure(0, weight=1)

# Sidebar menu (Left Navigation)
sidebar = ctk.CTkFrame(root, width=180, corner_radius=15, fg_color="gray17")
sidebar.grid(row=0, column=0, rowspan=4, sticky="nswe", padx=10, pady=10)

# Main content frame (right side, changes dynamically)
content_frame = ctk.CTkFrame(root, corner_radius=15, fg_color="gray13")
content_frame.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)

# Function to switch between pages
def show_page(page):
    # Clear current content
    for widget in content_frame.winfo_children():
        widget.destroy()
    
    if page == "Dashboard":
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
        
        stats_info = ctk.CTkLabel(content_frame, text="Users Online: 128\nActive Sessions: 45", font=("Arial", 16), text_color="white")
        stats_info.pack(pady=10)

# Function to change the theme based on selection
def change_theme(mode):
    mode = mode.lower()  # Convert the selected mode to lowercase
    if mode == "dark":
        ctk.set_appearance_mode("dark")
    elif mode == "light":
        ctk.set_appearance_mode("light")
    else:  # For "System" mode
        ctk.set_appearance_mode("system")
    
    # Optionally reinitialize UI after the theme change

# Start Progress
def start_progress():
    # Start the progress bar and simulate time for waffle preparation
    global progress_bar, estimated_time_label
    estimated_time = 30  # seconds for the estimated time to the next waffle
    estimated_time_label.configure(text=f"Estimated time to next waffle: {estimated_time}s")
    progress_bar.configure(mode="determinate", maximum=estimated_time, value=0)
    update_progress(estimated_time)

# Update progress bar
def update_progress(remaining_time):
    if remaining_time > 0:
        progress_bar.set(remaining_time)
        remaining_time -= 1
        estimated_time_label.configure(text=f"Estimated time to next waffle: {remaining_time}s")
        root.after(1000, update_progress, remaining_time)  # Update every second
    else:
        progress_bar.set(0)
        estimated_time_label.configure(text="Waffle ready!")

# Stop Progress
def stop_progress():
    # Reset the progress bar and estimated time
    global progress_bar, estimated_time_label
    progress_bar.configure(mode="indeterminate", value=0)
    estimated_time_label.configure(text="Estimated time to next waffle: 0s")

# Emergency stop logic with delay
def emergency_stop_action():
    global emergency_stop_state
    if emergency_stop_state == "STOPPED":
        emergency_stop_state = "PRESSED"  # Mark the first press as "PRESSED"
        btn_emergency.configure(fg_color="red", text="Emergency Stop Pressed")
    elif emergency_stop_state == "PRESSED":
        emergency_stop_state = "HOMING"  # Now set it to "HOMING" for second press
        btn_emergency.configure(fg_color="red", text="Homing")
        root.after(2000, reset_emergency_stop)  # Reset to "Emergency Stop" after 2 seconds

# Reset the emergency stop after 2 seconds
def reset_emergency_stop():
    global emergency_stop_state
    emergency_stop_state = "STOPPED"
    btn_emergency.configure(fg_color="red", text="Emergency Stop")

# Variable to keep track of the emergency stop state
emergency_stop_state = "STOPPED"

# Sidebar buttons with navigation functionality
btn_dashboard = ctk.CTkButton(sidebar, text="Dashboard", height=50, font=("Arial", 18), fg_color="gray35", command=lambda: show_page("Dashboard"))
btn_dashboard.pack(pady=10, padx=10, fill="x")

btn_settings = ctk.CTkButton(sidebar, text="Settings", height=50, font=("Arial", 18), fg_color="gray35", command=lambda: show_page("Settings"))
btn_settings.pack(pady=10, padx=10, fill="x")

btn_stats = ctk.CTkButton(sidebar, text="Stats", height=50, font=("Arial", 18), fg_color="gray35", command=lambda: show_page("Stats"))
btn_stats.pack(pady=10, padx=10, fill="x")

# Emergency Stop button
btn_emergency = ctk.CTkButton(sidebar, text="Emergency Stop", height=50, font=("Arial", 18), fg_color="red", command=emergency_stop_action)
btn_emergency.pack(pady=20, padx=10, fill="x")

# Exit button
btn_exit = ctk.CTkButton(sidebar, text="Exit", fg_color="yellow", height=50, font=("Arial", 18), command=root.quit)
btn_exit.pack(pady=10, padx=10, fill="x")

# Load default page
show_page("Dashboard")

# Run the application
root.mainloop()
