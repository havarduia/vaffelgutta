"""Test script for hand detection."""

from camera.vision import Vision
from camera.ai.hand_detection import HandDetector

def main():
    """Run hand detection test."""
    # Initialize vision system
    cam = Vision()

    # Initialize hand detector
    detector = HandDetector(cam.camera)

    # Run hand detection
    print("Starting hand detection...")
    detector.start()
    print("Hand detection complete.")

if __name__ == "__main__":
    main()