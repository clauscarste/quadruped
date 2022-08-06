
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=3)
#inspired by https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython

def set_angle(servo_angle1, servo_angle2, servo_angle3,ofsets):
    #calculatring angles with the ofset
    servo_angle1 = servo_angle1 - ofsets[0]
    servo_angle2 = servo_angle2 - ofsets[1]
    servo_angle3 = servo_angle3 - ofsets[2]

    # Setup:
    servokit.servo[0].actuation_range = 180
    servokit.servo[1].actuation_range = 180
    servokit.servo[2].actuation_range = 180
    kit.servo[0].set_pulse_width_range(800, 2000)
    kit.servo[1].set_pulse_width_range(800, 2000)
    kit.servo[2].set_pulse_width_range(800, 2000)

    kit.servo[0].angle = servo_angle1
    kit.servo[1].angle = servo_angle2
    kit.servo[2].angle = servo_angle3

set_angle(10,10,10,[0,0,0])


