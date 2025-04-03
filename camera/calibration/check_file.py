import numpy as np
import os

# Load the .npz file
data = np.load(os.path.expanduser('~/git/vaffelgutta/camera/camera_calibration.npz'))

# Print the keys in the file
print(data.files)
