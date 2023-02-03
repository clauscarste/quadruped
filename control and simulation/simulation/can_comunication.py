# goal is the this script will replace the can comunication script in all its functionalites and send the necesary data
# to the simulation and retrive the necesary date from the simulation

import numpy as np
import time


def dictionary():
    ###Dictionary###
    global state
    state = [0,0,0,0,0,0,0,0,0,0,0,0] #0 ist idle -  1 ist closed loop
    global position_setpoint
    position_setpoint = [0,0,0,0,0,0,0,0,0,0,0,0]
    global position_estimate
    position_estimate = [0,0,0,0,0,0,0,0,0,0,0,0]
    global velocity_estimate
    velocity_estimate = [0,0,0,0,0,0,0,0,0,0,0,0]
    global current_estimate
    current_estimate = [0,0,0,0,0,0,0,0,0,0,0,0]
    global force_estimate
    force_estimate = [0,0,0,0,0,0,0,0,0,0,0,0]
    global velocity_max_setpoint
    velocity_max_setpoint = [0,0,0,0,0,0,0,0,0,0,0,0]
    global current_max_setpoint
    current_max_setpoint = [0,0,0,0,0,0,0,0,0,0,0,0]
    global gyrodata
    gyrodata = [0,0,0,0,0,0,0,0,0] #angle xyz , angular velocity xzy , angular acceleration xyz
    global measured_force
    measured_force = [0,0,0,0,0,0,0,0,0,0,0,0]
    global torque_setpoint
    torque_setpoint = [0,0,0,0,0,0,0,0,0,0,0,0]
    global kpkv
    kpkv = [0,0]
    ###         ####
    global walk
    walk = False
    global jump
    jump = False
    global lower
    lower = False
    global set_all_motors_idle
    set_all_motors_idle = False
    global set_all_motors_closed_loop
    set_all_motors_closed_loop = False
    global increase_speed
    increase_speed = False
    global decrease_speed
    decrease_speed = False
    global incease_left
    incease_left = False
    global incease_right
    incease_right = False
    global currently_walking
    currently_walking = False
    global left_right_balance
    left_right_balance = 0
    global sepped_balance
    sepped_balance = 0
    global swaitchgraph
    swaitchgraph = 0
    global showtorque
    showtorque = True




















def set_closed_loop(msg_axis_id,closed_loop_attempt):
    #print("axis",msg_axis_id," is now in closed loop")
    state[msg_axis_id] = 1

def set_idle(msg_axis_id):
    state[msg_axis_id] = 0

def move_to(msg_axis_id,angle,ofsets,angle_limit,invert_axis):
    #check if angle is in bound - if its ouside bound bring it to bound border
    if angle > angle_limit:
        angle = angle_limit
    if invert_axis: #inversion is there to make axis behave like defined
        angle = -angle
    #angle = angle - ofsets
    if state[msg_axis_id] == 0:
        print("axis",msg_axis_id," is idle and just recived a command to move to",angle, "therefore it will stay at the current position")
        pass
    #covert angle to number
    gear_ratio = 6
    ange_number = (angle / 360)*gear_ratio
    position_setpoint[msg_axis_id] = ange_number
    #print("real",ange_number,"axis",msg_axis_id)

def clear_errors(msg_axis_id, data=[], format=''):
    pass

def is_bus_voltage_in_limit(battery_voltage_lower_limit,battery_voltage_upper_limit):
    voltage = can_get_voltage()
    if voltage == 0:
        return True
    if voltage < battery_voltage_lower_limit:
        print("battery voltage as gone outside of lower limit", battery_voltage_lower_limit,"V")
        return False
    elif voltage > battery_voltage_upper_limit:
        print("battery voltage as gone outside of upper limit", battery_voltage_upper_limit,"V")
        return False
    return True

def can_get_voltage(data=[], format='', RTR=True):
    return 20 #static vlaue for the simulation

def get_encoder_estimate(msg_axis_id, data=[], format='', RTR=True):
    a = position_estimate[msg_axis_id]
    b = velocity_estimate[msg_axis_id]
    return [a,b]

def get_iq(msg_axis_id, data=[], format='', RTR=True):
    return [current_estimate[msg_axis_id],current_estimate[msg_axis_id]]

def set_limits(msg_axis_id,current_max,velocity_max):
    current_max_setpoint[msg_axis_id] = current_max
    velocity_max_setpoint[msg_axis_id] = velocity_max

def setall_idle():
    for i in [0,1,2,3,4,5,6,7,8,9,10,11]:
        set_idle(i)
def setall_closed(attempts):
    for i in [0,1,2,3,4,5,6,7,8,9,10,11]:
        set_closed_loop(i,attempts)

