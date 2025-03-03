
import cv2
import time
# Import the classes
from aruco import Aruco
from coordinate_system import CoordinateSystem
from camera import Camera

def main():
    # Initialize the Aruco system
    try:
        aruco_system = Aruco()
    except RuntimeError as e:
        print(e)
        return

    print("Press 'q' or ESC to exit.")
    while True:

        results = aruco_system.estimate_pose()
      
        for cam_id, (image, transformations) in results.items():
            if image is not None:
                cv2.imshow(f"Camera {cam_id}", image)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
