import Jetson.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set the pin to read from (using GPIO 18 as an example)
button_pin = 18

# Set the pin as an input
GPIO.setup(button_pin, GPIO.IN)

try:
    print("Testing GPIO pin. Press and release the button.")
    while True:
        # Read the GPIO state and print it
        pin_state = GPIO.input(button_pin)
        if pin_state == GPIO.HIGH:
            print("Button is pressed! Pin state: HIGH")
        else:
            print("Button is released! Pin state: LOW")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted via Ctrl+C.")
finally:
    GPIO.cleanup()
