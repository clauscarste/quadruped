import math
import can
import cantools
import time
import os
import struct
#config
#in Odrive for all axis: odrv0.axisx.config.can.node_id = x
#set baudrate be sure that Odrive has the same - <odrv>.can.config.baud_rate = 250000
#os.system("sudo ip link set can0 up type can bitrate 250000")
#os.system("sudo ifconfig can0 txqueuelen 65536")

db = cantools.database.load_file("odrive-cansimple.dbc")
bus = can.Bus("can0", bustype="socketcan")


def set_closed_loop(msg_axis_id,closed_loop_attempt):
    data = db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
    msg = can.Message(arbitration_id=0x07 | msg_axis_id << 5, is_extended_id=False, data=data)

    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_closed_loop NOT sent!")

    for msg in bus:
        if msg.arbitration_id == 0x01 | msg_axis_id << 5:
            msg = db.decode_message('Heartbeat', msg.data)
            print(msg)
            if msg['Axis_State'] == 0x8:
                print("Axis has entered closed loop")
            else:
                print("Axis failed to enter closed loop - clearing errors and trying again ",closed_loop_attempt,"time(s)")
                clear_errors(msg_axis_id)
                if closed_loop_attempt <= 1:
                    set_closed_loop(msg_axis_id,closed_loop_attempt-1)
            break

def set_idle(msg_axis_id,set_idle_attempt):
    data = db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x01})
    msg = can.Message(arbitration_id=0x07 | msg_axis_id << 5, is_extended_id=False, data=data)

    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_idle NOT sent!")

    for msg in bus:
        if msg.arbitration_id == 0x01 | msg_axis_id << 5:
            msg = db.decode_message('Heartbeat', msg.data)
            if msg['Axis_State'] == 0x1:
                print("Axis has entered idle")
            else:
                print("Axis failed to enter set_idle - trying again ", set_idle_attempt,"time(s)")
                if set_idle_attempt <= 1:
                    set_idle(msg_axis_id, set_idle_attempt - 1)
            break

def move_to(msg_axis_id,angle,ofsets,angle_limit,invert_axis):
    #calculating angle with offset
    if angle < 0:
        angle = 0
    elif angle > angle_limit:
        angle = angle_limit
    if invert_axis: #inversion is there to make axis behave like defined
        angle = -angle
    angle = angle - ofsets

    #check if angle is in bound - if its ouside bound bring it to bound border

    #covert angle to number
    gear_ratio = 6
    ange_number = (angle / 360)*gear_ratio

    data = db.encode_message('Set_Input_Pos', {'Input_Pos': ange_number, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x00C, data=data, is_extended_id=False)

    try:
        bus.send(msg)
    except can.CanError:
        print("can_move_to NOT sent!")


def clear_errors(msg_axis_id, data=[], format=''):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x018), data=data_frame, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_clear_errors NOT sent!")

def is_bus_voltage_in_limit(battery_voltage_lower_limit,battery_voltage_upper_limit,can_get_voltage_attempt):
    voltage = can_get_voltage(12,can_get_voltage_attempt)
    if voltage < battery_voltage_lower_limit:
        print("battery voltage as gone outside of lower limit", battery_voltage_lower_limit,"V")
        return False
    elif voltage > battery_voltage_upper_limit:
        print("battery voltage as gone outside of upper limit", battery_voltage_upper_limit,"V")
        return False
    return True

def can_get_voltage(msg_axis_id,can_get_voltage_attempt, data=[], format='', RTR=True):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x17), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        bus.send(msg)
    except can.CanError:
        print("can_vbus NOT sent!")
    for msg in bus:
        if (msg.arbitration_id == (msg_axis_id << 5) + 0x17):
            msg = db.decode_message('Get_Vbus_Voltage', msg.data)
            return msg['Vbus_Voltage']
        else: # recursive call in case the first time no data is recived - message will be sent again to retry.
            print("no voltage data recived- trying again", can_get_voltage_attempt,"time(s)")
        if can_get_voltage_attempt <= 1:
            return(can_get_voltage(msg_axis_id, can_get_voltage_attempt - 1))
        break

def get_encoder_estimate(msg_axis_id,get_encoder_estimate_attempt, data=[], format='', RTR=True):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x009), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        bus.send(msg)
    except can.CanError:
        print("encoder_request NOT sent!")
    for msg in bus:
        if (msg.arbitration_id == (msg_axis_id << 5) + 0x009):
            msg = db.decode_message('Get_Encoder_Estimates', msg.data)
            return msg['Pos_Estimate'],msg['Vel_Estimate']
        else: # recursive call in case the first time no data is recived - message will be sent again to retry.
            print("no encoder data recived- trying again", get_encoder_estimate_attempt,"time(s)")
        if get_encoder_estimate_attempt <= 1:
            return(get_encoder_estimate(msg_axis_id, get_encoder_estimate_attempt - 1))
        break