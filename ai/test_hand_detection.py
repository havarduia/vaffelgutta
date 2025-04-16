from camera.vision import Vision
from hand_detection import HandDetector  # Replace with correct file if needed

if __name__ == "__main__":
    cam = Vision()
    detector = HandDetector(cam)
    detector.start()