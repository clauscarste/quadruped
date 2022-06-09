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


def set_closed_loop(msg_axis_id):
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
                print("Axis failed to enter closed loop")
            break

def set_idle(msg_axis_id):
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
                print("Axis failed to enter idle")
            break

def move_to(msg_axis_id,angle,ofsets,angle_limit):
    #calculating angle with offset
    angle = angle - ofsets
    #check if angle is in bound - if its ouside bound bring it to bound border
    if angle < 0:
        angle = 0
    elif angle > angle_limit:
        angle = angle_limit
    #covert angle to number
    ange_number = angle / 360

    data = db.encode_message('Set_Input_Pos', {'Input_Pos': ange_number, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x00C, data=data, is_extended_id=False)

    try:
        bus.send(msg)
    except can.CanError:
        print("can_move_to NOT sent!")

def set_limits(msg_axis_id,vel_limit,current_limit):
    data = db.encode_message('Set_Limits', {'Velocity_Limit': vel_limit, 'Current_Limit': current_limit})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x00F, is_extended_id=False, data=data)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_limits NOT sent!")

def set_pos_gain(msg_axis_id,pos_gain):
    data = db.encode_message('Set_Pos_Gain', {'Pos_Gain': pos_gain})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x1a, is_extended_id=False, data=data)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_pos_gain NOT sent!")

def set_vel_gains(msg_axis_id,vel_gain,vel_integ):
    data = db.encode_message('Set_Vel_gains', {'Vel_Gain': vel_gain, 'Vel_Integrator_Gain': vel_integ})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x1b, is_extended_id=False, data=data)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_vel_gains NOT sent!")


def encoder_estimates(msg_axis_id):
    while (1):
        msg = bus.recv()
        if (msg.arbitration_id == (msg_axis_id << 5) + 0x9):
            data = struct.unpack('ff', msg.data)
            return data[0]

def clear_errors(msg_axis_id, data=[], format=''):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x018), data=data_frame, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_clear_errors NOT sent!")

def is_bus_voltage_in_limit(battery_voltage_lower_limit,battery_voltage_upper_limit):
    msg_axis_id = 0
    voltage = 0
    if voltage(1) < battery_voltage_lower_limit | voltage(1) > battery_voltage_upper_limit:
        save_operation = False
        print("battery voltage as gone outside of lower limit", battery_voltage_lower_limit,"V")
        print("battery voltage as gone outside of upper limit", battery_voltage_upper_limit,"V")
        return False
    return True
