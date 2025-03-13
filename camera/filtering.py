from scipy.signal import savgol_filter # Found from stackoverflow: 
# https://stackoverflow.com/questions/52450681/how-can-i-use-smoothing-techniques-to-remove-jitter-in-pose-estimation

def smooth_data(pose_data, window_length=5, polyorder=2):
    """
    Apply Savitzky-Golay filter to smooth pose data.

    Parameters:
        pose_data (numpy.ndarray): The raw pose data to be smoothed.
        window_length (int): The length of the filter window (i.e., the number of coefficients). Must be a positive odd integer.
        polyorder (int): The order of the polynomial used to fit the samples. Must be less than window_length.

    Returns:
        numpy.ndarray: Smoothed pose data.
    """
    if window_length % 2 == 0:
        window_length += 1  # Window length must be odd.
    return savgol_filter(pose_data, window_length, polyorder, axis=0)