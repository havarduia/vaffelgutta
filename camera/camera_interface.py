from aruco import Aruco
from coordinate_system import CoordinateSystem
import cv2
from time import sleep

def main():
    try:
        # Initialize the Aruco instance and CoordinateSystem.
        ar = Aruco()
        cs = CoordinateSystem(ar)
    except Exception as e:
        print(f"Initialization error: {e}")
        return

    while True:
        print("\n1. Update Marker Length")
        print("2. Change Origin Offset")
        print("3. Show Processed Camera Feed")
        print("4. Print Transformations")
        print("5. Save Transformations")
        print("6. Exit")

        choice = input("Choose Input: ").strip()

        if choice == '1':
            # Update marker length.
            new_length = input("Enter new marker length (e.g., 0.048): ").strip()
            try:
                new_length_val = float(new_length)
                cs.update_marker(new_length_val)
                print(f"Marker length updated.")
            except ValueError:
                print("Invalid value.")
                
        elif choice == '2':
            # Update origin offset.
            new_offset = input("Enter new offset as e.g. (0.105,0.108,0): ").strip()
            try:
                offset_vals = tuple(map(float, new_offset.split(',')))
                if len(offset_vals) != 3:
                    raise ValueError
                cs.update_origin_offset(offset_vals)
                print(f"Origin offset updated to {offset_vals}.")
            except ValueError:
                print("Invalid offset format.")
                
        elif choice == '3':
            # Show processed camera feed from estimate_pose.
            print("Displaying camera feeds. Press q to exit.")
            while True:
                # Get the processed images and transformations.
                results = ar.estimate_pose()
                # Display each camera's processed image.
                for cam_id, (image, transformations) in results.items():
                    if image is not None:
                        cv2.imshow(f"Camera {cam_id} (Processed)", image)
                # Exit if 'q' is pressed.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.destroyAllWindows()
            
        elif choice == '4':
            # Print transformations from the coordinate system.
            results = cs.transform_origin()
            if results is None:
                print("No transformations available. (Ensure origin marker with ID 0 is detected.)")
            else:
                for marker_id, transform in results.items():
                    print(f"\nMarker ID: {marker_id}")
                    print(transform)
                    
        elif choice == '5':
            print("Transformations saving... ")
            for i in range(0, 1000):
                sleep(0.25)
                cs.save_transformation()
            print(8*"\n")
            print("Saving process done... ")
                
            
            
        elif choice == '6':
            print("Exiting... ")
            break
            
        else:
            print("Invalid choice ")

if __name__ == '__main__':
    main()
