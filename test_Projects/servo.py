import Jetson.GPIO as GPIO

import time

# SETUP 🛠️
servo_pin = 33   # change based on your Jetson board layout
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM INIT 🔥
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz = 20ms period
pwm.start(0)

def set_angle(angle):
    # 0° -> 2.5% duty, 180° -> 12.5% duty (servo specific)
    duty = 2.5 + (angle / 180.0) * 10
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    pwm.ChangeDutyCycle(0)  # Stop signal to prevent jitter

# USAGE 💃
set_angle(0)     # Look left
set_angle(90)    # Center
set_angle(180)   # Right

# TEARDOWN 🧹
pwm.stop()
GPIO.cleanup()
