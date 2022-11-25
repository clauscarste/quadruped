#https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
import can_comunication
#fct to use return a array with deg_x / deg_y / deg_z / gyro_x / gyro_y / gyro z - where deg is the angle in degrees - and gyro the change of that angle in deg/s
def read_mpu():
    return can_comunication.gimbal
