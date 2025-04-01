import cv2
import numpy as np
import matplotlib.pyplot as plt

# Generate an ArUco marker
def generate_aruco(marker_id=0, marker_size=200, dictionary=cv2.aruco.DICT_4X4_50):
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    # Using drawMarker instead of generateImageMarker for current versions
    marker = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker, 1)
    return marker

# Overlay a grid that fits the 4x4 marker
def overlay_grid(image, n_cells=5):
    # Compute cell size assuming the image is square.
    cell_size = image.shape[0] // n_cells
    img_with_grid = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    
    # Draw vertical and horizontal grid lines at cell boundaries
    for i in range(1, n_cells):
        pos = i * cell_size
        cv2.line(img_with_grid, (pos, 0), (pos, image.shape[0]), (255, 0, 0), 1)
        cv2.line(img_with_grid, (0, pos), (image.shape[1], pos), (255, 0, 0), 1)
    return img_with_grid

# Apply Otsu's thresholding
def apply_otsu_threshold(image):
    _, thresh_img = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh_img

# Main Execution
marker_id = 2
marker_size = 200
aruco_marker = generate_aruco(marker_id, marker_size)

# Use a 4-cell grid for the 4x4 marker
marker_with_grid = overlay_grid(aruco_marker, n_cells=4)
otsu_marker = apply_otsu_threshold(aruco_marker)

# Display the results
fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].imshow(marker_with_grid, cmap='gray')
ax[0].set_title("ArUco Marker with 4x4 Grid")
ax[0].axis("off")

ax[1].imshow(otsu_marker, cmap='gray')
ax[1].set_title("Otsu's Thresholding")
ax[1].axis("off")

plt.show()
