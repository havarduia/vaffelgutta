from camera.init_camera import initalize_system 

def second_call_camera():
    _, __, coords = initalize_system()
    coords.start(25)
    print("second")


def main():
    camera, aruco, coords = initalize_system()
    for _ in range(10):
        print("first")
        coords.start(25)
        second_call_camera()


if __name__ == "__main__":
    main()