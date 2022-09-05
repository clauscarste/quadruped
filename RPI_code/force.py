import math
import numpy as np
import can_comunication


def inverse_and_transpose(m):
    return np.transpose(np.linalg.inv(m))

def force_calculation(leg_id, leg_parameters):
    # convert leg id to msg axis
    msg_axis_id = [leg_id * 3, leg_id * 3 + 1, leg_id * 3 + 2]

    # get angle and current
    gear_ratio = 6
    angle = np.array([can_comunication.get_encoder_estimate(msg_axis_id[0])[0] / gear_ratio * 360,
                      can_comunication.get_encoder_estimate(msg_axis_id[1])[0] / gear_ratio * 360,
                      can_comunication.get_encoder_estimate(msg_axis_id[2])[0] / gear_ratio * 360])
    current = np.array([abs(can_comunication.get_iq(msg_axis_id[0])[1]), abs(can_comunication.get_iq(msg_axis_id[1])[1]),abs(can_comunication.get_iq(msg_axis_id[1])[1])])

    # caclulate torque and current
    torque = -0.00221823 * current * current + 0.27497038 * current + 0.09294292
    o1 = np.radians(angle[0])
    o2 = np.radians(angle[1])
    o3 = np.radians(angle[2])
    l1 = leg_parameters[0]
    l2 = leg_parameters[1]
    l3 = leg_parameters[2]
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


