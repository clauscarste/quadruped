
#import can_thread                                                                                                #uncomment

#import kinematics_spine                #not implemented in simulation and not used in real word currently

#general imports
from __future__ import division
import time
import numpy as np
import kinematics_legs
import walking
import jumping

#############################  uncomment real world and simulation here and in the kinematics_legs.py  ################
#for real world testing
#from realworld import temprature_readout
#from realworld import can_comunication

#for simulation
from simulation import temprature_readout
from simulation import can_comunication
#can_comunication.dictionary()
# Config File Import
import configparser
def main_loop():
    config_obj = configparser.ConfigParser()
    config_obj.read("/Users/claus/PycharmProjects/quadruped/control and simulation/configfile.ini")

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

    #dynamic values changed by joystick / keyboard


    #fixed values for now
    xm = 0
    ym = 0
    zm = 0
    yaw = 0
    pich= 0
    roll= 0

    ######Config end #######



    #### Inital steps after startup####
    # Spine setup to zero position
    #kinematics_spine.inverse_kinematics_spine(0,0,distance_center_of_spine_to_rope_m,servo_ofset,max_angle,spine_length,pully_radius,norma_rope_length)

    # test that can works and encoders give good reading - also fill in libary
    for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]:
        can_comunication.get_encoder_estimate(i)
    for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]:
        print(can_comunication.get_encoder_estimate(i), "     this is ", i)

    # set motor closed loop
    can_comunication.setall_closed(closed_loop_attempt)
    #Set motor to inital positon



    ###### Main loop ######
    while save_operation == True:
         # chck for save operation (temprature and battery voltage)
        if can_comunication.is_bus_voltage_in_limit(battery_voltage_lower_limit,battery_voltage_upper_limit) is False or temprature_readout.is_temp_in_limit(temprature_limit) is False:
            save_operation = False
            can_comunication.setall_idle()
            print("not save")

        if can_comunication.walk == True and can_comunication.currently_walking == False:
            can_comunication.currently_walking = True
            can_comunication.walk = False
        if can_comunication.walk == True and can_comunication.currently_walking:
            can_comunication.currently_walking = False
            can_comunication.walk = False
        if can_comunication.jump == True:
            can_comunication.jump = False
            jumping.jump()
        if can_comunication.lower == True:
            can_comunication.lower = False
            jumping.jump()
        if can_comunication.set_all_motors_idle == True:
            can_comunication.set_all_motors_idle = False
            can_comunication.setall_idle()
        if can_comunication.set_all_motors_closed_loop == True:
            can_comunication.set_all_motors_closed_loop = False
            can_comunication.setall_closed(closed_loop_attempt)
        if can_comunication.increase_speed == True:
            can_comunication.increase_speed = False
            sepped_balance = sepped_balance + 1
        if can_comunication.decrease_speed == True:
            can_comunication.decrease_speed = False
            sepped_balance = sepped_balance - 1
        if can_comunication.incease_left == True:
            can_comunication.incease_left = False
            left_right_balance = left_right_balance - 1
        if can_comunication.incease_right == True:
            can_comunication.incease_right = False
            left_right_balance = left_right_balance + 1
        ## implement speed and direction ### with left_right_balance and sepped_balance

        if can_comunication.currently_walking == True:
            walking.walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
                                      acceleration_stance, deceleration_stance, speed_flight, acceleration_flight,
                                      deceleration_flight,
                                      yaw, pich, roll, xm, ym, zm, robot_length, robot_with, leg_parameters, ofset, limit,
                                      invert_axis, leg_config)
































    ##Testing fuctionality

    #setting closed and idle
    #can_comunication.set_closed_loop(3,2)
    #can_comunication.set_closed_loop(4,2)
    #can_comunication.set_closed_loop(5,2)
    #can_comunication.setall_idle()
    #can_comunication.setall_closed(5)


    #Plot walking curve
    #walking.ploting(walking.curve_stance(1,step_lentgh,stance_max_height,neutral_height,speed_stance,acceleration_stance,deceleration_stance))
    #walking.ploting(walking.curve_flight(1,step_lentgh,flight_max_heigth,neutral_height,speed_flight,acceleration_flight,deceleration_flight))


    #test inverse kinematics
    #print(kinematics_legs.yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z))


    #test walking sequence
    #walking.walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
     #                    acceleration_stance, deceleration_stance, speed_flight, acceleration_flight, deceleration_flight,
      #                   yaw, pich, roll, xm, ym, zm, robot_length,robot_with,leg_parameters, ofset, limit, invert_axis, leg_config)


    #Test Jump
    #upper = -0.2999
    #land = -0.2
    #lower = -0.1
    #delay = 0.01
    #jumping.jump(land,upper,lower,delay,leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm, robot_length, robot_with)


    #Test force response
    #for i in np.arange(-0.15, -0.299, -0.001):
    #    time.sleep(0.001)
    #    kinematics_legs.inverse_kinematics_legs(0, 0, i, 0.1, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm,
    #                            ym, zm, robot_length,robot_with)










#######for real world ######
#main_loop()
"""
config_obj = configparser.ConfigParser()
config_obj.read("/Users/claus/PycharmProjects/quadruped/control and simulation/configfile.ini")

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


walking.ploting(walking.curve_stance(1,step_lentgh,stance_max_height,neutral_height,speed_stance,acceleration_stance,deceleration_stance))
walking.ploting(walking.curve_flight(1,step_lentgh,flight_max_heigth,neutral_height,speed_flight,acceleration_flight,deceleration_flight))
"""