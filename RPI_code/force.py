import can_comunication
import time
import numpy as np

def force_calculation(msg_axis_id):
    current = can_comunication.get_iq(msg_axis_id,get_iq_attempt)[1]
    torque = -0.00221823*current*current+0.27497038*current+0.09294292
    force = torque*0.15
    return abs(force) #lever for the knee and hip are slighlty different that is nont looked at here

def contact_detection(leg_id,contact_f):
        #as if only one motor is checked there could be the case, that it is not detected.
        #for exaple if hip is straight down there is almost no force on motor.
        #also if one motor the gears are slightly stuck it wont detect
        if force_calculation(leg_id*3+2)>contact_f: #and force_calculation(leg_id*3+1, get_iq_attempt)>contact_f:
            return True

###test that leg kinematics move leg to position until force is to high. Then it stops.
#can_comunication.set_closed_loop(5,1)
#can_comunication.set_idle(5,1)
#can_comunication.get_encoder_estimate(5,1)
can_comunication.move_to(5,0,0,180,1)
for i in (np.arange(0,90,0.1)):
    time.sleep(0.5)
    can_comunication.move_to(5, i, 0, 180, 1)
    if contact_detection(1,1,0.1):
        print("contact")
        can_comunication.move_to(5, 0, 0, 180, 1)
        break
