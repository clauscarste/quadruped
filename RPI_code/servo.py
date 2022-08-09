
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
#inspired by https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython

def map(value, leftMin, leftMax, rightMin, rightMax):
    left = leftMax - leftMin
    right = rightMax - rightMin
    scale = float(value - leftMin) / float(left)
    return rightMin + (scale * right)

def set_angle(servo_angle1, servo_angle2, servo_angle3,ofsets):
    #calculatring angles with the ofset
    servo_angle1 = servo_angle1 +135 + ofsets[0] #+135 sets zero point of servo to center of ratiational maxima
    servo_angle2 = servo_angle2 +135 + ofsets[1]
    servo_angle3 = servo_angle3 + 135 + ofsets[2]

    # Setup:
    kit.servo[0].actuation_range = 270
    kit.servo[1].actuation_range = 270
    kit.servo[2].actuation_range = 270
    kit.servo[0].set_pulse_width_range(470, 2550)
    kit.servo[1].set_pulse_width_range(470, 2550)
    kit.servo[2].set_pulse_width_range(470, 2550)

    #map to invert
    servo_angle1 = map(servo_angle1,0,270,270,0)

    kit.servo[0].angle = servo_angle1 #rechts
    kit.servo[1].angle = servo_angle2 #links
    kit.servo[2].angle = servo_angle3#top

#set_angle(10,10,10,[0,0,0])


