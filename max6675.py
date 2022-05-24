#https://github.com/archemius/MAX6675-Raspberry-pi-python/blob/master/max6675.py
#edited to not require input arguments for fuctions

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

cs_no = 22
# set pin number for communicate with MAX6675
def set_pin():
    UNIT = 0
    CS = 22
    SCK = 18
    SO = 16
    global sck
    sck = SCK
    global so
    so = SO
    global unit
    unit = UNIT

    GPIO.setup(CS, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(SCK, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(SO, GPIO.IN)


def read_temp():
    GPIO.output(cs_no, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(cs_no, GPIO.HIGH)
    time.sleep(0.22)

    GPIO.output(cs_no, GPIO.LOW)
    GPIO.output(sck, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(sck, GPIO.LOW)
    Value = 0
    for i in range(11, -1, -1):
        GPIO.output(sck, GPIO.HIGH)
        Value = Value + (GPIO.input(so) * (2 ** i))
        GPIO.output(sck, GPIO.LOW)

    GPIO.output(sck, GPIO.HIGH)
    error_tc = GPIO.input(so)
    GPIO.output(sck, GPIO.LOW)

    for i in range(2):
        GPIO.output(sck, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(sck, GPIO.LOW)

    GPIO.output(cs_no, GPIO.HIGH)

    if unit == 0:
        temp = Value
    if unit == 1:
        temp = Value * 0.25
    if unit == 2:
        temp = Value * 0.25 * 9.0 / 5.0 + 32.0

    if error_tc != 0:
        return -cs_no
    else:
        return temp


GPIO.cleanup()
