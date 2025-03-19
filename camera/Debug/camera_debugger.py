import cv2
import numpy as np
import logging
import os
import tkinter as tk
from tkinter import messagebox
from camera.init_camera import initalize_system
np.set_printoptions(precision=2)
import time
class ArucoDebugger:
    def __init__(self, aruco, coord_sys):
        self.coord_sys = coord_sys
        self.aruco = aruco
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    
    def check_camera_feed(self):
        while True:
            image = self.aruco.camera.get_image()
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
        corners, ids = self.aruco._aruco_detection()
        if ids is None:
            logging.warning("No markers detected.")
            messagebox.showwarning("Warning", "No markers detected.")
        else:
            logging.info(f"Detected markers: {ids}")
            messagebox.showinfo("Info", f"Detected markers: {ids}")
    
    def visualize_markers(self):
        while True:
            image = self.aruco.camera.get_image()
            if image is None:
                logging.error("No image received from camera.")
                messagebox.showerror("Error", "No image received from camera.")
                break
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco.detector.detectMarkers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                transformations = self.aruco.estimate_pose()
                for i in range(len(ids)):
                    tag_id = int(ids[i])  # Convert NumPy array to integer
                    if tag_id in transformations:
                        T = transformations[tag_id]
                        rvec, _ = cv2.Rodrigues(T[:3, :3])
                        tvec = T[:3, 3].reshape(-1, 1)
                        cv2.drawFrameAxes(image, self.aruco.camera.get_calibration()[0], self.aruco.camera.get_calibration()[1], rvec, tvec, 0.05)
                
            cv2.imshow("Detected Markers", image)
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to close
                break
        cv2.destroyAllWindows()
    

    def debug_pose_estimation(self):
        while True:
            # Clear the terminal before printing the new transformation
            os.system("cls" if os.name == "nt" else "clear")

            transformation = self.coord_sys.transformation_origin_to_tag()
            print("Transformation Matrix:\n", transformation)

            # Allow exit with ESC key
            if cv2.waitKey(1) & 0xFF == 27:  
                break


def create_gui(debugger):
    root = tk.Tk()
    root.title("Aruco Debugger GUI")
    
    frame = tk.Frame(root, padx=10, pady=10)
    frame.pack(padx=20, pady=20)
    
    tk.Button(frame, text="Check Camera Feed", command=debugger.check_camera_feed, width=25).pack(pady=5)
    tk.Button(frame, text="Validate Marker Detection", command=debugger.validate_marker_detection, width=25).pack(pady=5)
    tk.Button(frame, text="Visualize Markers", command=debugger.visualize_markers, width=25).pack(pady=5)
    tk.Button(frame, text="Debug Pose Estimation", command=debugger.debug_pose_estimation, width=25).pack(pady=5)
    
    root.mainloop()

if __name__ == "__main__":
    camera, aruco, coord_sys, config_loader = initalize_system()
    debugger = ArucoDebugger(aruco, coord_sys)
    create_gui(debugger)