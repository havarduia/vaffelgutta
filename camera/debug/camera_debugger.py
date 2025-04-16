import cv2
import logging
import customtkinter as ctk
import tkinter.messagebox as messagebox
from camera.vision import *
import numpy as numphy
from camera.config.configloader import ConfigLoader
from robot.tools.file_manipulation import Jsonreader
from PIL import Image
class ArucoDebugger:
    def __init__(self, vision):
        self.vision = vision
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    
    def check_camera_feed(self):
        while True:
            image = self.vision._get_image()
            if image is None:
                logging.error("Camera feed is not available. Check camera connection.")
                messagebox.showerror("Error", "Camera feed is not available.")
                break
            
            logging.info("Camera feed is working.")
            cv2.imshow("Camera Feed", image)
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to close
                break
        cv2.destroyAllWindows()
    
    def validate_marker_detection(self):
        corners, ids = self.vision._aruco_detection()
        if ids is None:
            logging.warning("No markers detected.")
            messagebox.showwarning("Warning", "No markers detected.")
        else:
            logging.info(f"Detected markers: {ids}")
            messagebox.showinfo("Info", f"Detected markers: {ids}")
    
    def visualize_markers(self):
        vision.run(True, "all")

    def debug_pose_estimation(self):
        try:
            while True: 
                pose, _ = self.vision._estimate_pose()
                tags = self.vision._coordinatesystem(pose)
                for tag, T in tags.items():
                    print(f"Tag ID: {tag} Transformation: \n{numphy.array(T)}\n")
        except KeyboardInterrupt:
            print("\nProcess interrupted by user. Exiting... ")

def create_gui(debugger):
    # Set appearance and theme
    ctk.set_appearance_mode("System")  # Options: "System", "Dark", "Light"
    ctk.set_default_color_theme("blue")  # Options: "blue", "green", "dark-blue"

    # Initialize the window
    root = ctk.CTk()
    root.title("Aruco Debugger")
    root.geometry("700x500")
    root.resizable(False, False)

    # Load and set background image
    bg_label = ctk.CTkLabel(root, text="")
    bg_label.place(x=0, y=0, relwidth=1, relheight=1)

    # Main frame over background
    frame = ctk.CTkFrame(root, corner_radius=20, fg_color="transparent")
    frame.place(relx=0.5, rely=0.5, anchor="center")

    # Title label
    title_label = ctk.CTkLabel(
        frame,
        text="Aruco Marker Debugging Tool",
        font=ctk.CTkFont(size=22, weight="bold")
    )
    title_label.pack(pady=(10, 20))

    # Button style helper
    def make_button(text, command):
        return ctk.CTkButton(
            frame,
            text=text,
            command=command,
            width=300,
            height=40,
            font=ctk.CTkFont(size=16)
        )

    # Buttons
    make_button("Check Camera Feed", debugger.check_camera_feed).pack(pady=10)
    make_button("Validate Marker Detection", debugger.validate_marker_detection).pack(pady=10)
    make_button("Visualize Markers", debugger.visualize_markers).pack(pady=10)
    make_button("Debug Pose Estimation", debugger.debug_pose_estimation).pack(pady=10)

    # Appearance toggle (optional)
    def toggle_theme():
        current = ctk.get_appearance_mode()
        ctk.set_appearance_mode("Dark" if current == "Light" else "Light")

    theme_button = ctk.CTkButton(
        frame,
        text="Toggle Theme",
        command=toggle_theme,
        width=160,
        font=ctk.CTkFont(size=14),
        fg_color="gray",
        hover_color="darkgray"
    )
    theme_button.pack(pady=(20, 0))

    root.mainloop()

if __name__ == "__main__":
    vision = Vision()
    debugger = ArucoDebugger(vision)
    create_gui(debugger)
