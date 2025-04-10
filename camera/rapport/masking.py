from camera.init_camera import initalize_system as init
import cv2
import numpy as np
from time import sleep

def main():
    
    camera,_,_ = init()
    sleep(2)
    
    image = camera.get_image()
    if image is None:
        print("Error: Could not read the image.")
        exit()

    # Create a blank mask with the same dimensions as the image (single channel)
    mask = np.zeros(image.shape[:2], dtype="uint8")

    # Define the center of the image and the size of the square mask
    x_center, y_center = image.shape[1] // 2, image.shape[0] // 2
    square_size = 150  # side length of the square

    # Calculate the top-left and bottom-right coordinates for the square
    top_left = (x_center - square_size // 2, y_center - square_size // 2)
    bottom_right = (x_center + square_size // 2, y_center + square_size // 2)

    # Draw a filled white square (rectangle) on the mask
    cv2.rectangle(mask, top_left, bottom_right, 255, -1)

    # Apply the mask to the image using bitwise AND
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Display the original image, mask, and masked image
    cv2.imshow("Original Image", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Masked Image", masked_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


        
        
    
if __name__ == "__main__":
    main()
