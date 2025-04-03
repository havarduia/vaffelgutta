import cv2

# Board parameters (example for A4, adjust as needed)
num_cols = 5          # number of squares along the width
num_rows = 7          # number of squares along the height
square_length = 0.038  # 38 mm in meters
marker_length = 0.026  # about 70% of square length

# Get the predefined dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Create the board using the new API (note: tuple ordering is (num_cols, num_rows))
board = cv2.aruco.CharucoBoard((num_cols, num_rows), square_length, marker_length, aruco_dict)
# (Optional) For legacy behavior, uncomment the next line:
# board.setLegacyPattern(True)

# Define desired physical size to nearly fill A4 with margins (in pixels)
# For example, let’s assume effective board area ~190x277 mm at 300 DPI:
dpi = 300
mm_to_inch = 0.0393701
width_in = 190 * mm_to_inch
height_in = 277 * mm_to_inch
board_size_px = (int(width_in * dpi), int(height_in * dpi))

# Generate the board image (this returns a NumPy array)
board_image = board.generateImage(board_size_px, marginSize=15)

# Save as PNG
png_filename = "charuco_board_A4.png"
cv2.imwrite(png_filename, board_image)
