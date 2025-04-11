# Import only if if running on Jetson
from sys import modules as sysmodules
if "Jetson.GPIO" in sysmodules:
    import Jetson.GPIO as GPIO

from time import sleep
from threading import Thread, Event


def monitor_emergency_stop(emergency_stop_state: Event):
    """ Function to monitor GPIO button in a separate thread. """
    # Set the GPIO mode
    GPIO.setmode(GPIO.BOARD)
    button_pin = 18  # Define button pin
    # Set the pin as an input
    GPIO.setup(button_pin, GPIO.IN)
            
    while True:
        pin_state = GPIO.input(button_pin)
        if pin_state == GPIO.LOW:
            emergency_stop_state.set()
            break
        sleep(0.1)  # Prevent CPU overuse


def run_emergency_stop_monitor(emergency_stop_state: Event):
    if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
        # Start GPIO monitoring in a separate thread
        gpio_thread = Thread(
            target=monitor_emergency_stop, daemon=True, args = [emergency_stop_state])
        gpio_thread.start()
    return
