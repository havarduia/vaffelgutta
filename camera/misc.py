import yaml
import os
from scipy.signal import savgol_filter

def smooth_data(pose_data, window_length=5, polyorder=2):
    """
    Apply Savitzky-Golay filter to smooth pose data.

    Parameters:
        pose_data (numpy.ndarray): The raw pose data to be smoothed.
        window_length (int): The length of the filter window (must be a positive odd integer).
        polyorder (int): The order of the polynomial used to fit the samples (must be less than window_length).

    Returns:
        numpy.ndarray: Smoothed pose data.
    """
    # Ensure window_length is odd.
    if window_length % 2 == 0:
        window_length += 1

    # Adjust window_length if the data length is too short.
    n_samples = pose_data.shape[0]
    if n_samples < window_length:
        # Use the largest odd number less than or equal to n_samples.
        window_length = n_samples if n_samples % 2 == 1 else n_samples - 1
        # Also ensure that polyorder is less than window_length.
        polyorder = min(polyorder, window_length - 1)

    return savgol_filter(pose_data, window_length, polyorder, axis=0)

class ConfigLoader:
    def __init__(self, config_path=os.path.expanduser('~/git/vaffelgutta/camera/config.yaml')):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
    
    def get(self, key, default=None):
        return self.config.get(key, default)
    
def print_blue(message):
    BLUE = "\033[94m"   # ANSI code for bright blue text
    RESET = "\033[0m"   # ANSI code to reset the text color to default
    print(f"{BLUE}{message}{RESET}")

def print_error(message):
    RED = "\033[91m"   # ANSI code for bright red text
    RESET = "\033[0m"  # ANSI code to reset text color to default
    print(f"{RED}{message}{RESET}")
    


