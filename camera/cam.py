from camera.aruco import Aruco
from camera.coordinate_system import CoordinateSystem
from time import sleep
def main():
    try:
        # Initialize the Aruco instance and CoordinateSystem.
        ar = Aruco()
        cs = CoordinateSystem(ar)
    except Exception as e:
        print(f"Initialization error: {e}")
        return
    

    for i in range(0,1000):
        cs.save_transformation()
        sleep(0.25)
        
    return