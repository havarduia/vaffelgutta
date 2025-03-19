from camera.realsense import Camera
from camera.aruco import Aruco
from camera.coordinatesystem import CoordinateSystem
from camera.Config.misc import ConfigLoader

# Added type annotations for initialize-system (line 7) -AT
def initalize_system()-> tuple[Camera, Aruco, CoordinateSystem]:
    """Initializes the camera, ArUco detector, and coordinate system."""
    config_loader = ConfigLoader()
    camera = Camera(config_loader)
    aruco = Aruco(camera,config_loader)
    coord_sys = CoordinateSystem(aruco,config_loader)
    return camera, aruco, coord_sys, config_loader