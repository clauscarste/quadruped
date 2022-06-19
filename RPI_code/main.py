#libaries
from __future__ import division

#created .py files
import servo
import temprature_readout
import can_comunication
import kinematics_legs
import kinematics_spine

#Config File Import
import configparser
config_obj = configparser.ConfigParser()
config_obj.read("C:\PythonTutorials\configfile.ini")
dbparam = config_obj["postgresql"]
print(dbparam)

######Config start#########

#Temprature and Voltage limits of motors and battery
save_operation = True
temprature_limit = 70
battery_voltage_lower_limit = 15
battery_voltage_upper_limit = 22

#servo_initial_angles and ofset (negative defined)
servo_1_inital_angle = 0
servo_2_inital_angle = 0
servo_3_inital_angle = 0
servo_ofset = [0, 0, 0]

#Motor parameters
motor_inital_xyz = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]] #motor inital angels after startup
motor_ofset = [0,0,0,0,0,0,0,0,0,0,0,0] #ofset of each motor +calculated by helper function to have zero degree position at a the same point for every motor regardless of the instalation orientation of the encoder
angle_limit = [180,180,180,180,180,180,180,180,180,180,180,180] #limit of each motor seen from the set zero degree position +calculated by helper function
invert_axis = [False,False,False,False,False,False,False,False,False,False,False,False] #inversion of each motor to achive desired positve defined movement direction
leg_parameters = [0.1,0.6,0.6] #length of joints pivot, hip, knee

#Number of times to retry after failing to send message over can bus
can_get_voltage_attempt = 1
set_idle_attempt = 1
closed_loop_attempt = 1
get_encoder_estimate_attempt = 1

######Config end #########







#Servo setup
servo.set_angle(servo_1_inital_angle, servo_2_inital_angle, servo_3_inital_angle, servo_ofset)
#Motor_setup
for i in range(13):
    can_comunication.set_closed_loop(i,closed_loop_attempt)
for i in range(4):
    kinematics_legs.inverse_kinematics_legs(i,motor_inital_xyz[i],leg_parameters,motor_ofset,angle_limit,invert_axis)##++move to initial position with inverse kinematics
#inverse kinematics

while True:
    #chck for save operation (temprature and battery voltage)
    if can_comunication.is_bus_voltage_in_limit(battery_voltage_lower_limit, battery_voltage_upper_limit,can_get_voltage_attempt) == False or temprature_readout.is_temp_in_limit(temprature_limit) == False:
        save_operation = False

    #Shut down if save operation is no longer granted or if shut down is wanted- by setting all axis to idle
    if (save_operation == False):
        for i in range(13):
            can_comunication.set_idle(i,set_idle_attempt)











