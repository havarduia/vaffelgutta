from camera.realsense import Camera
from camera.aruco import Aruco
from camera.coordinatesystem import CoordinateSystem
from camera.camera_config_loader import ConfigLoader

def initalize_system():
    """Initializes the camera, ArUco detector, and coordinate system."""
    config_loader = ConfigLoader()
    camera = Camera(config_loader)
    aruco = Aruco(camera,config_loader)
    coord_sys = CoordinateSystem(aruco,config_loader)
    return camera, aruco, coord_sys