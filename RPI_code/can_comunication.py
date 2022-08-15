import time
import can
import cantools
import struct
import can_thread
from threading import Thread
can_thread.dictionary()

#config necesary for Odirve
#in Odrive for all axis: odrv0.axisx.config.can.node_id = x
#set baudrate be sure that Odrive has the same - <odrv>.can.config.baud_rate = 250000
#config necesary for RPI
#os.system("sudo ip link set can0 up type can bitrate 250000")
#os.system("sudo ifconfig can0 txqueuelen 65536")

db = cantools.database.load_file("odrive-cansimple.dbc")
bus = can.Bus("can0", bustype="socketcan")
bus2 = can.Bus("can1", bustype="socketcan")


def set_closed_loop(msg_axis_id,closed_loop_attempt):
    data = db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
    msg = can.Message(arbitration_id=0x07 | msg_axis_id << 5, is_extended_id=False, data=data)

    try:
        if msg_axis_id in [3,4,5,6,7,8]:
            bus.send(msg)
        else:
            bus2.send(msg)

    except can.CanError:
        print("can_set_closed_loop NOT sent!")
    time.sleep(0.4)
    if can_thread.loop_state[msg_axis_id] == 0x8:
        print("Axis has entered closed loop")
    else:
        print("Axis failed to enter closed loop - clearing errors and trying again ",closed_loop_attempt,"time(s)")
        clear_errors(msg_axis_id)
        if closed_loop_attempt >= 1:
            set_closed_loop(msg_axis_id,closed_loop_attempt-1)

def set_idle(msg_axis_id):
    data = db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x01})
    msg = can.Message(arbitration_id=0x07 | msg_axis_id << 5, is_extended_id=False, data=data)

    try:
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("can_set_idle NOT sent!")
    time.sleep(0.4)
    if can_thread.loop_state[msg_axis_id] == 0x1:
        print("Axis has entered idle")
    else:
        print("Axis failed to enter set_idle")

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
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("can_move_to NOT sent!")


def clear_errors(msg_axis_id, data=[], format=''):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x018), data=data_frame, is_extended_id=False)
    try:
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("can_clear_errors NOT sent!")

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
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((4 << 5) | 0x17), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        bus.send(msg)
    except can.CanError:
        print("can_vbus NOT sent!")
    return can_thread.loop_state[13]


def get_encoder_estimate(msg_axis_id, data=[], format='', RTR=True):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x009), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("encoder_request NOT sent!")
    return can_thread.encoder_estimate[msg_axis_id]



def get_iq(msg_axis_id, data=[], format='', RTR=True):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x014), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("iq_request NOT sent!")
    return can_thread.iq[msg_axis_id]

def set_limits(msg_axis_id,current_max,velocity_max):
    data = db.encode_message('Set_Limits', {'Current_Limit': current_max, 'Velocity_Limit': velocity_max,})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x00F, data=data, is_extended_id=False)
    try:
        if msg_axis_id in [3, 4, 5, 6, 7, 8]:
            bus.send(msg)
        else:
            bus2.send(msg)
    except can.CanError:
        print("can_move_to NOT sent!")

def setall_idle():
    for i in [3,4,5,6,7,8]:
        set_idle(i)
def setall_closed(attempts):
    for i in [3, 4, 5, 6, 7, 8]:
        set_closed_loop(i,attempts)

# create new threads
t1 = Thread(target=can_thread.get_all_updates)
t2 = Thread(target=can_thread.get_all_updates2)

# start the threads
t1.start()
t2.start()

