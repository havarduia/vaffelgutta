import numpy as numphy # <3
from camera.aruco import Aruco  
from camera.camera import Camera
import os
import time

class CoordinateSystem:
    def __init__(self, aruco_instance, marker_length=0.048, origin_offset=(0.105, 0.108, 0)):
        self.aruco = aruco_instance  # Use the provided instance.
        self.marker_length = marker_length
        self.origin_transform = None 
        self.origin_offset = numphy.array(origin_offset)
        
    def update_marker(self, marker_length):
        self.marker_length = marker_length
        return
    
    def update_origin_offset(self, origin_offset: numphy.array):
        self.origin_offset = origin_offset
        return
    
    def transform_origin(self):
        results = self.aruco.estimate_pose(self.marker_length)
        relative_transformations = {}

        for cam_id, (image, transformations) in results.items():
            if transformations:
                origin_detected = False
                # First pass to find the origin marker (ID 0)
                for marker_id, transform in transformations:
                    if marker_id == 0:
                        # Create offset transformation matrix
                        offset_transform = numphy.eye(4)
                        offset_transform[:3, 3] = self.origin_offset
                        # Apply offset to the origin marker's transform
                        self.origin_transform = transform @ offset_transform
                        origin_detected = True
                        break  # Exit loop after finding origin marker

                if not origin_detected:
                    print("Origin marker not detected!")
                    return None

                # Second pass to compute relative transformations
                for marker_id, transform in transformations:
                    if marker_id != 0:
                        relative_transform = numphy.linalg.inv(self.origin_transform) @ transform
                        relative_transformations[marker_id] = relative_transform

        return relative_transformations
    
    def save_transformation(self):
        current_transformations = self.transform_origin()
        if not current_transformations:
            print("No transformations to save.")
            return
        
        current_transformations_lists = {
            marker_id: transform.tolist() 
            for marker_id, transform in current_transformations.items()
        }
        
        file_dir = os.path.expanduser("~/git/vaffelgutta/robot/assets/positions")
        file_path = os.path.join(file_dir, "camera_readings.py")
        os.makedirs(file_dir, exist_ok=True)
        
        existing_data = {}
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    content = f.read()
                    namespace = {}
                    exec(content, namespace)
                    # Collect existing tag variables (tag_X where X is the marker ID)
                    for name in namespace:
                        if name.startswith('tag_'):
                            marker_id_str = name[4:]  # Remove 'tag_' prefix
                            if marker_id_str.isdigit():
                                marker_id = int(marker_id_str)
                                existing_data[marker_id] = namespace[name]
            except Exception as e:
                print(f"Error reading existing transformations: {e}. Starting fresh.")
                existing_data = {}
        
        # Update with current transformations
        existing_data.update(current_transformations_lists)
        
        try:
            with open(file_path, 'w') as f:
                # Write each transformation as a separate variable
                for marker_id in sorted(existing_data.keys()):
                    transform = existing_data[marker_id]
                    f.write(f"tag_{marker_id} = ([\n")
                    for row in transform:
                        # Format each element to 8 decimal places
                        formatted_elements = [f"{elem:.8f}" for elem in row]
                        # Create the row string with proper spacing
                        f.write(f"    [ {', '.join(formatted_elements)} ],\n")
                    f.write("])\n\n")
        except Exception as e:
            print(f"Error saving transformations: {e}")
