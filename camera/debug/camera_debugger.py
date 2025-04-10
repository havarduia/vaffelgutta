import cv2
import logging
import customtkinter as ctk
import tkinter.messagebox as messagebox
from camera.vision import *
import numpy as numphy
from camera.config.misc import ConfigLoader

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
        while True:
            _, img = self.vision._estimate_pose()
            self.vision.show_image(img)
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to close
                break
        cv2.destroyAllWindows()

    def debug_pose_estimation(self):
        try:
            while True: 
                pose, _ = self.vision._estimate_pose()
                tags = self.vision._transformation_to_tag(pose)
                for tag, T in tags.items():
                    print_blue(f"Tag ID: {tag} Transformation: \n{numphy.array(T)}\n")
        except KeyboardInterrupt:
            print_error("\nProcess interrupted by user. Exiting gracefully.")

def create_gui(debugger):
    # Initialize the customtkinter window
    root = ctk.CTk()
    root.title("Aruco Debugger GUI")
    root.geometry("320x200")
    
    frame = ctk.CTkFrame(root, corner_radius=10)
    frame.pack(padx=20, pady=20, fill="both", expand=True)
    
    btn_check_feed = ctk.CTkButton(
        frame,
        text="Check Camera Feed",
        command=debugger.check_camera_feed,
        width=200
    )
    btn_check_feed.pack(pady=5)
    
    btn_validate_detection = ctk.CTkButton(
        frame,
        text="Validate Marker Detection",
        command=debugger.validate_marker_detection,
        width=200
    )
    btn_validate_detection.pack(pady=5)
    
    btn_visualize_markers = ctk.CTkButton(
        frame,
        text="Visualize Markers",
        command=debugger.visualize_markers,
        width=200
    )
    btn_visualize_markers.pack(pady=5)
    
    btn_debug_pose = ctk.CTkButton(
        frame,
        text="Debug Pose Estimation",
        command=debugger.debug_pose_estimation,
        width=200
    )
    btn_debug_pose.pack(pady=5)
    
    root.mainloop()

if __name__ == "__main__":
    config = ConfigLoader()
    vision = Vision(config)
    debugger = ArucoDebugger(vision)
    create_gui(debugger)
