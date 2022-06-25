from time import sleep, perf_counter
from threading import Thread
import can_comunication
import can
import cantools
bus = can.Bus("can0", bustype="socketcan")
db = cantools.database.load_file("odrive-cansimple.dbc")


###Dictionary###
global loop_state
global bus_voltage
global encoder_estimate
global get_iq
###         ###

def get_all_updates():
    while True:
        # Decoding messages and saving to Dictionary
        start_time = perf_counter()
        for msg in bus:
            if (msg.arbitration_id == (5 << 5) + 0x17):
                msg_voltage = db.decode_message('Get_Vbus_Voltage', msg.data)
                bus_voltage = msg_voltage['Vbus_Voltage']

            for msg_axis_id in range(0,13):
                if msg.arbitration_id == 0x01 | msg_axis_id << 5:
                    msg_heart = db.decode_message('Heartbeat', msg.data)
                    loop_state[msg_axis_id] = msg_heart['Axis_State']

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x009):
                    msg_encoder = db.decode_message('Get_Encoder_Estimates', msg.data)
                    encoder_estimate[msg_axis_id] = [msg_encoder['Pos_Estimate'], msg_encoder['Vel_Estimate']]

                if (msg.arbitration_id == (msg_axis_id << 5) + 0x014):
                    msg_iq = db.decode_message('Get_Iq', msg.data)
                    get_iq[msg_axis_id] = [msg_iq['Iq_Measured'], msg_iq['Iq_Setpoint']]

        end_time = perf_counter()
        print(f'It took {end_time - start_time: 0.2f} second(s) to complete.')

# create new threads
t1 = Thread(target=get_all_updates)
# start the threads
t1.start()
