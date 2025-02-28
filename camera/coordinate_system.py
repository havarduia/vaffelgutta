import numpy as np
from aruco import Aruco  # Import your Aruco class

class CoordinateSystem:
    def __init__(self, marker_length=0.048):
        self.aruco = Aruco()  # Aruco detector
        self.marker_length = marker_length
        self.origin_transform = None  # To store the transformation of ID 0

    def compute_relative_transformations(self):
        """
        Computes transformations of all Aruco markers relative to the origin (ID 0).
        """
        results = self.aruco.estimate_pose(self.marker_length)
        relative_transformations = {}

        for cam_id, (image, transformations) in results.items():
            if transformations:
                # Extract the transformation of the origin marker (ID 0)
                for marker_id, transform in transformations:
                    if marker_id == 0:
                        self.origin_transform = transform
                        break  # Stop once we find ID 0

                if self.origin_transform is None:
                    print("Origin marker not detected!")
                    return None

                # Compute relative transformations
                for marker_id, transform in transformations:
                    if marker_id != 0:
                        relative_transform = np.linalg.inv(self.origin_transform) @ transform
                        relative_transformations[marker_id] = relative_transform

        return relative_transformations

def main():
    coord_sys = CoordinateSystem()
    rel_transforms = coord_sys.compute_relative_transformations()
    
    if rel_transforms:
        for marker_id, transform in rel_transforms.items():
            print(f"Marker {marker_id} relative to ID 0:\n{transform}\n")
    
# Example usage:
if __name__ == "__main__":
    main()