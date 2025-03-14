# Import only if if running on Jetson
from sys import modules as sysmodules
if "Jetson.GPIO" in sysmodules:
    import Jetson.GPIO as GPIO

from time import sleep
from threading import Thread
from typing import Callable

def run_emergency_stop_monitor(return_function: Callable):
    if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
        # **Start GPIO monitoring in a separate thread**
        gpio_thread = Thread(
            target=monitor_emergency_stop, daemon=True, args = [return_function])
        gpio_thread.start()
    return

def monitor_emergency_stop(return_function: Callable):
    """ Function to monitor GPIO button in a separate thread. """
    # Set the GPIO mode
    GPIO.setmode(GPIO.BOARD)
    button_pin = 18  # Define button pin
    # Set the pin as an input
    GPIO.setup(button_pin, GPIO.IN)
            
    while True:
        pin_state = GPIO.input(button_pin)
        if pin_state == GPIO.LOW:
            return_function(slow = True)
            break
        sleep(0.1)  # Prevent CPU overuse
