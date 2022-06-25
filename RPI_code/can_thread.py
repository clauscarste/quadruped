from time import sleep, perf_counter
from threading import Thread
import can_comunication
import can
import cantools
bus = can.Bus("can0", bustype="socketcan")
db = cantools.database.load_file("odrive-cansimple.dbc")




def get_all_updates():
    ###Dictionary###
    global loop_state
    loop_state = [0,0,0,0,0,0,0,0,0,0,0,0]
    global bus_voltage
    bus_voltage = -1
    global encoder_estimate
    encoder_estimate = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    global get_iq
    get_iq = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    ###         ###
    while True:
        # Decoding messages and saving to Dictionary
        start_time = perf_counter()
        for msg in bus:
            if (msg.arbitration_id == (5 << 5) + 0x17):
                try:
                    msg_voltage = db.decode_message('Get_Vbus_Voltage', msg.data)
                    bus_voltage = msg_voltage['Vbus_Voltage']
                except:
                    x = 0

            for msg_axis_id in range(0,13):
                if msg.arbitration_id == 0x01 | msg_axis_id << 5:
                    try:
                        msg_heart = db.decode_message('Heartbeat', msg.data)
                        loop_state[msg_axis_id] = msg_heart['Axis_State']
                    except:
                        x = 0

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x009):
                    try:
                        msg_encoder = db.decode_message('Get_Encoder_Estimates', msg.data)
                        encoder_estimate[msg_axis_id] = [msg_encoder['Pos_Estimate'], msg_encoder['Vel_Estimate']]
                    except:
                        x = 0

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x014):
                    try:
                        msg_iq = db.decode_message('Get_Iq', msg.data)
                        get_iq[msg_axis_id] = [msg_iq['Iq_Measured'], msg_iq['Iq_Setpoint']]
                    except:
                        x = 0
            print("out of for msg")
            break

        end_time = perf_counter()
        print(f'It took {end_time - start_time: 0.2f} second(s) to complete.')
        print(loop_state)

# create new threads
t1 = Thread(target=get_all_updates)
# start the threads
t1.start()
