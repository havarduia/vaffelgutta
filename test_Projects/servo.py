import Jetson.GPIO as GPIO

import time

# SETUP 🛠️
servo_pin = 33   # change based on your Jetson board layout
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM INIT 🔥
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz = 20ms period
pwm.start(0)



try:
    while True:
        inn = int(input("set new angel: "))
        if inn ==-1:
            break
        elif inn <= 100:
            pwm.ChangeDutyCycle(inn)
finally:
    # TEARDOWN 🧹
    pwm.stop()
    GPIO.cleanup()


