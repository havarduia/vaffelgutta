import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from coordinate_system import CoordinateSystem 

# Laget fra greg, bare for å se at coordinate_system.py faktisk gjør jobben riktig, takk greg.


def plot_marker_positions(rel_transforms):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the origin marker at (0,0,0) for reference
    ax.scatter(0, 0, 0, color='black', marker='o', s=100, label='Origin')
    ax.text(0, 0, 0, "0", color='black')
    
    for marker_id, transform in rel_transforms.items():
        # Extract the translation vector (4x4 matrix)
        pos = transform[:3, 3]
        ax.scatter(pos[0], pos[1], pos[2], label=f'Marker {marker_id}')
        ax.text(pos[0], pos[1], pos[2], f'{marker_id}', color='red')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Hagle')
    ax.legend()
    plt.show()

def main():
    # Instantiate the coordinate system and compute relative transformations
    coord_sys = CoordinateSystem()
    rel_transforms = coord_sys.compute_relative_transformations()
    
    if rel_transforms:
        for marker_id, transform in rel_transforms.items():
            print(f"Origin to Marker {marker_id}:\n{transform}\n")
        # Plot the positions in 3D
        plot_marker_positions(rel_transforms)
    else:
        print("arker ID 0 is not detected.")

if __name__ == "__main__":
    main()
