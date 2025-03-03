import cv2
import time
import threading
import queue

from aruco import Aruco
from coordinate_system import CoordinateSystem
from camera import Camera


terminal_queue = queue.Queue()
periodic_save_thread = None
periodic_save_stop_event = threading.Event()

def terminal_interface():
    while True:
        print("\nOptions:")
        print("1: Change marker length")
        print("2: Toggle periodic saving of transformations")
        print("3: Toggle streaming on all cameras")
        print("4: Change origin offset")
        print("5: Print current transformations")
        print("q: Quit")
        user_input = input("Enter option: ").strip()
        terminal_queue.put(user_input)
        if user_input.lower() == "q":
            break

def periodic_save(coord_sys, stop_event, interval=5):
    while not stop_event.is_set():
        coord_sys.save_transformation()
        print("Periodic save performed.")
        # Wait for the interval or until stop_event is set.
        stop_event.wait(interval)

def process_terminal_input(option, aruco_system, coord_sys):
    """Process the option selected by the user."""
    global periodic_save_thread, periodic_save_stop_event
    if option == "1":
        try:
            new_length = float(input("Enter new marker length (in meters): "))
            coord_sys.marker_length = new_length
            print(f"Marker length updated to {new_length}")
        except ValueError:
            print("Invalid input for marker length.")
    elif option == "2":
        # Toggle periodic saving of transformations.
        if periodic_save_thread is None or not periodic_save_thread.is_alive():
            # Start periodic saving.
            periodic_save_stop_event.clear()
            periodic_save_thread = threading.Thread(
                target=periodic_save, args=(coord_sys, periodic_save_stop_event), daemon=True
            )
            periodic_save_thread.start()
            print("Started periodic saving of transformations.")
        else:
            # Stop periodic saving.
            periodic_save_stop_event.set()
            periodic_save_thread.join()
            periodic_save_thread = None
            print("Stopped periodic saving of transformations.")
    elif option == "3":
        # Toggle streaming for each camera.
        for serial, info in aruco_system.cameras.items():
            cam = info["obj"]
            if cam is not None:
                if cam.isStreaming:
                    cam.stop_streaming()
                    print(f"Stopped streaming for camera {serial}")
                else:
                    cam.start_streaming()
                    print(f"Started streaming for camera {serial}")
    elif option == "4":
        try:
            offset_str = input("Enter new origin offset as three comma-separated values (e.g., 0.1,0.1,0): ")
            parts = offset_str.split(',')
            if len(parts) != 3:
                raise ValueError("Please enter exactly three values.")
            new_offset = tuple(float(p.strip()) for p in parts)
            coord_sys.origin_offset = new_offset
            print(f"Origin offset updated to {new_offset}")
        except ValueError as e:
            print(f"Invalid input for origin offset: {e}")
    elif option == "5":
        transformations = coord_sys.do_transformations()
        print("Current transformations:")
        if transformations:
            for marker_id, transform in transformations.items():
                print(f"Marker {marker_id}:")
                print(transform)
        else:
            print("No valid transformations found.")
    elif option.lower() == "q":
        return "quit"
    else:
        print("Unknown option.")

def main():
    global periodic_save_thread, periodic_save_stop_event

    try:
        aruco_system = Aruco()
    except RuntimeError as e:
        print(e)
        return

    # Init the coordinate system.
    coord_sys = CoordinateSystem()

    # Start the interface in a separate daemon thread.
    terminal_thread = threading.Thread(target=terminal_interface, daemon=True)
    terminal_thread.start()

    print("Press 'q' in the terminal or 'q'/ESC in the camera window to exit.")

    while True:
        try:
            option = terminal_queue.get_nowait()
            result = process_terminal_input(option, aruco_system, coord_sys)
            if result == "quit":
                break
        except queue.Empty:
            pass

        # Get images and pose estimations.
        results = aruco_system.estimate_pose()

        # Display each camera's feed.
        for cam_id, (image, transformations) in results.items():
            if image is not None:
                cv2.imshow(f"Camera {cam_id}", image)

        # Exit if 'q' or ESC is pressed in the OpenCV window.
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cv2.destroyAllWindows()
    
    # Ensure the periodic saving thread is properly stopped.
    if periodic_save_thread is not None and periodic_save_thread.is_alive():
        periodic_save_stop_event.set()
        periodic_save_thread.join()

    print("Exiting program.")

if __name__ == "__main__":
    main()
