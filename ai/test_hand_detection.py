from camera.config.misc import ConfigLoader
from camera.vision import Vision
from hand_detection import HandDetector  # Replace with correct file if needed

if __name__ == "__main__":
    config_loader = ConfigLoader()
    cam = Vision(config_loader)
    detector = HandDetector(cam)
    detector.start()