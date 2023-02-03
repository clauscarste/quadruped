# not used anymore
import math
import numpy as np
from simulation import can_comunication


def inverse_and_transpose(m):
    return np.transpose(np.linalg.inv(m))

def force_calculation():
    leg_id = 0
    leg_parameters = [0.1,0.15,0.15]
    # convert leg id to msg axis
    msg_axis_id = [leg_id * 3, leg_id * 3 + 1, leg_id * 3 + 2]
    angle1 = can_comunication.position_setpoint[leg_id * 3]
    angle2 = can_comunication.position_setpoint[leg_id * 3 + 1]
    angle3 = can_comunication.position_setpoint[leg_id * 3 + 2]

    # get current and ajust for roation direction
    array_temp = 0
    current = np.array([0.0,0.0,0.0])
    for i in msg_axis_id:
        temp = can_comunication.current_estimate[i]
        if i in [1,2,3,4,5,9]:
            temp = -temp
        current[array_temp] = temp
        array_temp = array_temp+1
    # caclulate torque and current
    #torque = -0.00221823 * current * current + 0.27497038 * current + 0.09294292
    torque = current
    l1 = leg_parameters[0]
    l2 = leg_parameters[1]
    l3 = leg_parameters[2]
    o1 = angle1
    o2 = angle2
    o3 = angle3
    # jacobian calcualted with matlab
    jacobian = np.array([[l2 * np.cos(o1) * np.cos(o2) - l1 * np.sin(o1) + l3 * np.cos(o1) * np.cos(o2) * np.cos(
        o3) - 1.0 * l3 * np.cos(o1) * np.sin(o2) * np.sin(o3),
                          - l2 * np.sin(o1) * np.sin(o2) - 1.0 * l3 * np.cos(o2) * np.sin(o1) * np.sin(
                              o3) - l3 * np.cos(o3) * np.sin(o1) * np.sin(o2),
                          - l3 * np.cos(o2) * np.sin(o1) * np.sin(o3) - 1.0 * l3 * np.cos(o3) * np.sin(o1) * np.sin(
                              o2)],
                         [l1 * np.cos(o1) + 1.0 * l2 * np.cos(o2) * np.sin(o1) + 1.0 * l3 * np.cos(o2) * np.cos(
                             o3) * np.sin(o1) - l3 * np.sin(o1) * np.sin(o2) * np.sin(o3),
                          1.0 * l2 * np.cos(o1) * np.sin(o2) + l3 * np.cos(o1) * np.cos(o2) * np.sin(
                              o3) + 1.0 * l3 * np.cos(o1) * np.cos(o3) * np.sin(o2),
                          1.0 * l3 * np.cos(o1) * np.cos(o2) * np.sin(o3) + l3 * np.cos(o1) * np.cos(o3) * np.sin(o2)],
                         [0, - l3 * np.cos(o2 + o3) - l2 * np.cos(o2), -l3 * np.cos(o2 + o3)]])
    return inverse_and_transpose(jacobian).dot(torque)



def contact_detection(leg_id, contact_f,leg_parameters):
    if abs(force_calculation(leg_id,leg_parameters)[1]) > contact_f:
        return True


