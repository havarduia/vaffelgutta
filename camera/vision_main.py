from camera.realsense import Camera
from camera.aruco import Aruco
from camera.coordinatesystem import CoordinateSystem
from camera.camera_config_loader import ConfigLoader                
        
def initalize_system():
    config_loader = ConfigLoader()
    camera = Camera(config_loader)
    aruco = Aruco()
    coord_sys = CoordinateSystem(config_loader)
    return camera, aruco, coord_sys

def main():
    camera, aruco, coord_sys = initalize_system()   
    cv2.namedWindow("Hagle", cv2.WINDOW_NORMAL) 

    while True:
        # Update pose estimation
        coord_sys.save_to_json(25)     
        image = camera.get_image()
        if image is not None:
            cv2.imshow("Hagle", image)
        else:
            print("Hagle")
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()