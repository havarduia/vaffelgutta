import Jetson.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)
channel = 15

GPIO.setup(channel, GPIO.IN)

while True:
    input = GPIO.input(15)
    print(input)
    if not input: break
GPIO.cleanup()