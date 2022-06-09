
from __future__ import division
from numpy import interp
import time
import Adafruit_PCA9685
#https://github.com/adafruit/Adafruit_Python_PCA9685/blob/master/examples/simpletest.py


def set_angle(servo_angle1, servo_angle2, servo_angle3,ofsets):
    #calculatring angles with the ofset
    servo_angle1 = servo_angle1 - ofsets[0]
    servo_angle2 = servo_angle2 - ofsets[1]
    servo_angle3 = servo_angle3 - ofsets[2]
    pwm = Adafruit_PCA9685.PCA9685()

    #converting desired angle to pwm signal using interp from numpy
    servo_min = 100
    servo_max = 600
    m = (interp([servo_angle1,servo_angle2,servo_angle3],[0,180],[servo_min,servo_max]))
    pwm.set_pwm(0,0,int(round(m[0],0)))
    pwm.set_pwm(1,0,int(round(m[1],0)))
    pwm.set_pwm(2,0,int(round(m[2],0)))


set_angle(10,10,10,[0,0,0])