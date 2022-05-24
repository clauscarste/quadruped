#libaries
from __future__ import division

#created .py files
import servo
import temprature_readout
import can_comunication
import kinematics_legs
import kinematics_spine

#Initial setup and defenition of  parameters
save_operation = True
temprature_limit = 70
battery_voltage_lower_limit = 15
battery_voltage_upper_limit = 22

#servo_initial_angles and ofset (negative)
servo_1_inital_angle = 0
servo_2_inital_angle = 0
servo_3_inital_angle = 0
servo_ofset = [0, 0, 0]


#motor inital angle and ofset (negative)
motor_inital_angle = [0,0,0,0,0,0,0,0,0,0,0,0]
motor_ofset = [0,0,0,0,0,0,0,0,0,0,0,0]
leg_parameters = [1,1,1]
#Servo setup
servo.set_angle(servo_1_inital_angle, servo_2_inital_angle, servo_3_inital_angle, servo_ofset)
#Motor_setup
for i in range(13):
    can_comunication.set_closed_loop(i)
    can_comunication.move_to(i, motor_inital_angle[i], motor_ofset[i])

#inverse kinematics
kinematics_spine.inverse_kinematics_spine(1, 1, 1, servo_ofset)
kinematics_legs.inverse_kinematics_legs(1, 1, 1, 1, leg_parameters, motor_ofset)

while True:
    #chck for save operation (temprature and battery voltage)
    if can_comunication.is_bus_voltage_in_limit(battery_voltage_lower_limit, battery_voltage_upper_limit) == False or temprature_readout.is_temp_in_limit(temprature_limit) == False:
        save_operation = False

    #Shut down if save operation is no longer granted or if shut down is wanted- by setting all axis to idle
    if (save_operation == False):
        for i in range(13):
            can_comunication.set_idle(i)








