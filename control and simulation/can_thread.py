from time import sleep, perf_counter
import can
import cantools
db = cantools.database.load_file("odrive-cansimple.dbc")

def dictionary():
    ###Dictionary###
    global loop_state
    loop_state = [0,0,0,0,0,0,0,0,0,0,0,0,0]
    #global bus_voltage
    #bus_voltage = 0
    global encoder_estimate
    encoder_estimate = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    global iq
    iq = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    ###         ###

def get_all_updates():
    bus = can.Bus("can0", bustype="socketcan")

    while True:
        # Decoding messages and saving to Dictionary
        start_time = perf_counter()
        for msg in bus:
            if (msg.arbitration_id == (4 << 5) + 0x17):
                try:
                    msg_voltage = db.decode_message('Get_Vbus_Voltage', msg.data)
                    #bus_voltage = msg_voltage['Vbus_Voltage']
                    loop_state[12] = msg_voltage['Vbus_Voltage']

                except ValueError:
                    pass

            for msg_axis_id in range(0,13):
                if msg.arbitration_id == 0x01 | msg_axis_id << 5:
                    try:
                        msg_heart = db.decode_message('Heartbeat', msg.data)
                        loop_state[msg_axis_id] = msg_heart['Axis_State']
                    except ValueError:
                        pass

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x009):
                    try:
                        msg_encoder = db.decode_message('Get_Encoder_Estimates', msg.data)
                        encoder_estimate[msg_axis_id] = [msg_encoder['Pos_Estimate'], msg_encoder['Vel_Estimate']]
                    except ValueError:
                        pass

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x014):
                    try:
                        msg_iq = db.decode_message('Get_Iq', msg.data)
                        iq[msg_axis_id] = [msg_iq['Iq_Measured'], msg_iq['Iq_Setpoint']]
                    except ValueError:
                        pass
            break

        end_time = perf_counter()
        #print(f'It took {end_time - start_time: 0.2f} second(s) to complete.')


def get_all_updates2():
    bus = can.Bus("can1", bustype="socketcan")

    while True:
        # Decoding messages and saving to Dictionary
        start_time = perf_counter()
        for msg in bus:
            for msg_axis_id in range(0,13):
                if msg.arbitration_id == 0x01 | msg_axis_id << 5:
                    try:
                        msg_heart = db.decode_message('Heartbeat', msg.data)
                        loop_state[msg_axis_id] = msg_heart['Axis_State']
                    except ValueError:
                        pass

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x009):
                    try:
                        msg_encoder = db.decode_message('Get_Encoder_Estimates', msg.data)
                        encoder_estimate[msg_axis_id] = [msg_encoder['Pos_Estimate'], msg_encoder['Vel_Estimate']]
                    except ValueError:
                        pass

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x014):
                    try:
                        msg_iq = db.decode_message('Get_Iq', msg.data)
                        iq[msg_axis_id] = [msg_iq['Iq_Measured'], msg_iq['Iq_Setpoint']]
                    except ValueError:
                        pass
            break

        end_time = perf_counter()
        #print(f'It took {end_time - start_time: 0.2f} second(s) to complete.')




#used in can comunication cumunication for voltage
#in for for curent
#in setup halper für encoder

