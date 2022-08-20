# libaries
from __future__ import division

# created .py files
import time

#import servo
#import temprature_readout
import can_comunication
import kinematics_legs
import walking
#import kinematics_spine

# Config File Import
import configparser

config_obj = configparser.ConfigParser()
config_obj.read("configfile.ini")

save_operation_limits = config_obj["save_operation_limits"]
servo_config = config_obj["servo_config"]
motor_config = config_obj["motor_config"]
can_retry_amount = config_obj["can_retry_amount"]

# Temprature and Voltage limits of motors and battery
save_operation = True
temprature_limit = float(save_operation_limits["temprature_limit"])
battery_voltage_lower_limit = float(save_operation_limits["battery_voltage_lower_limit"])
battery_voltage_upper_limit = float(save_operation_limits["battery_voltage_upper_limit"])

# servo_initial_angles and ofset (negative defined)
servo_1_inital_angle = float(servo_config["servo_1_inital_angle"])
servo_2_inital_angle = float(servo_config["servo_2_inital_angle"])
servo_3_inital_angle = float(servo_config["servo_3_inital_angle"])
servo_ofset = list(map(float, (servo_config["servo_ofset"]).split()))
distance_center_of_spine_to_rope_m = float(servo_config["distance_center_of_spine_to_rope_m"])
spine_length = float(servo_config["spine_length"])
max_angle = float(servo_config["max_angle"])
pully_radius = float(servo_config["pully_radius"])
norma_rope_length = float(servo_config["norma_rope_length"])

# Motor parameters
motor_inital_x = list(map(float, (motor_config["motor_inital_x"]).split()))
motor_inital_y = list(map(float, (motor_config["motor_inital_y"]).split()))
motor_inital_z = list(map(float, (motor_config["motor_inital_z"]).split()))
ofset = list(map(float, (motor_config["ofset"]).split()))
limit = list(map(float, (motor_config["limit"]).split()))
invert_axis = (list(map(int, (motor_config["invert_axis"]).split())))
leg_parameters = list(map(float, (motor_config["leg_parameters"]).split()))
robot_length = float(motor_config["robot_length"])
robot_with = float(motor_config["robot_with"])
contact_f = float(motor_config["contact_f"])

leg_config = float(motor_config["leg_config"])
step_lentgh = float(motor_config["step_lentgh"])
stance_max_height = float(motor_config["stance_max_height"])
flight_max_heigth = float(motor_config["flight_max_heigth"])
neutral_height = float(motor_config["neutral_height"])
speed_stance = int(motor_config["speed_stance"])
speed_flight = int(motor_config["speed_flight"])
acceleration_stance = float(motor_config["acceleration_stance"])
deceleration_stance = float(motor_config["deceleration_stance"])
acceleration_flight = float(motor_config["acceleration_flight"])
deceleration_flight = float(motor_config["deceleration_flight"])

# Number of times to retry after failing to send message over can bus
closed_loop_attempt = int(can_retry_amount["closed_loop_attempt"])


xm = 0
ym = 0
zm = 0

leg_id= 0
x = 0
y = -0.2
z = 0.1

yaw = 0
pich= 0
roll= 0

######Config end #######


# Servo setup to zero position
#kinematics_spine.inverse_kinematics_spine(0,0,distance_center_of_spine_to_rope_m,servo_ofset,max_angle,spine_length,pully_radius,norma_rope_length)

# test that can works and encoders give good reading - also fill in libary
# time.sleep(0.5)
# can_comunication.can_get_voltage()
# for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]:
#     can_comunication.get_encoder_estimate(i)
# for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]:
#     print(can_comunication.get_encoder_estimate(i), "     this is ", i)
#

# set motor closed loop
#can_comunication.setall_closed(closed_loop_attempt)
#can_comunication.set_closed_loop(0,1)
#time.sleep(5)

# Set motor to inital positon
#for i in range(4):
#    kinematics_legs.inverse_kinematics_legs(leg_id, motor_inital_x[i], motor_inital_y[i], motor_inital_z[i], leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm,
#                            ym, zm, robot_length,robot_with)

# while save_operation == True:
#     # chck for save operation (temprature and battery voltage)
#     if can_comunication.is_bus_voltage_in_limit(battery_voltage_lower_limit,
#                                                 battery_voltage_upper_limit) is False or temprature_readout.is_temp_in_limit(
#             temprature_limit) is False:
#         save_operation = False
#         print("not save")
#
#     # Shut down if save operation is no longer granted or if shut down is wanted- by setting all axis to idle
#     if (save_operation == False):
#         can_comunication.setall_idle()
#
#     ###place walking or jumping or spine movement calls here.
#     while True:
#         walking.walking_sequence(step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance
#                                  ,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight)
#     ###
#


##Testinf fuctionality

#plotting walkinc curves
#can_comunication.set_closed_loop(3,2)
#can_comunication.set_closed_loop(4,2)
#can_comunication.set_closed_loop(5,2)
#can_comunication.setall_idle()
#walking.ploting(walking.curve_stance(1,step_lentgh,stance_max_height,neutral_height,speed_stance,acceleration_stance,deceleration_stance))
#walking.ploting(walking.curve_flight(1,step_lentgh,flight_max_heigth,neutral_height,speed_flight,acceleration_flight,deceleration_flight))

#test inverse kinematics
#print(kinematics_legs.yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z))
#kinematics_legs.inverse_kinematics_legs(leg_id, x, y, z, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm,
 #                           ym, zm, robot_length,robot_with)

#test walking
#walking.walk_stance(leg_id, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
 #               deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
  #              robot_length,robot_with)
#walking.walk_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
 #               deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
  #              robot_length,robot_with)

#test walking sequence
walking.walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
                     acceleration_stance, deceleration_stance, speed_flight, acceleration_flight, deceleration_flight,
                     yaw, pich, roll, xm, ym, zm, robot_length,robot_with,leg_parameters, ofset, limit, invert_axis, leg_config)