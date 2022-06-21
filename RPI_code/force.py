import can_comunication
import time

def force_calculation(msg_axis_id,get_iq_attempt):
    current = can_comunication.get_iq(msg_axis_id,get_iq_attempt)[1]
    torque = -0.00221823*current*current+0.27497038*current+0.09294292
    force = torque*0.15
    return force #lever for the knee and hip are slighlty different that is nont looked at here

def contact_detection(leg_id,get_iq_attempt,contact_f):
        #as if only one motor is checked there could be the case, that it is not detected.
        #for exaple if hip is straight down there is almost no force on motor.
        #also if one motor the gears are slightly stuck it wont detect
        if force_calculation(leg_id*3+1, get_iq_attempt)>contact_f and force_calculation(leg_id*3+2, get_iq_attempt)>contact_f:
            return True
