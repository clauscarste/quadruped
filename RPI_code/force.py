import math
import numpy as np
#import can_comunication


def inverse_and_transpose(m):
    return np.transpose(np.linalg.inv(m))

def can_testing_simulation(a):
    return 1 + a * 0.1

def force_calculation(leg_id,leg_parameters):
    #convert leg id to msg axis
    msg_axis_id = [leg_id * 3, leg_id * 3 + 1,leg_id * 3 + 2]

    #get angle and current
    gear_ratio = 6
    angle = np.array([can_testing_simulation(msg_axis_id[0])/gear_ratio*360, can_testing_simulation(msg_axis_id[1])/gear_ratio*360,can_testing_simulation(msg_axis_id[2])/gear_ratio*360])
    #position = np.array([can_comunication.get_encoder_estimate(msg_axis_id[0])[0]/gear_ratio*360, can_comunication.get_encoder_estimate(msg_axis_id[1])[0]/gear_ratio*360,can_comunication.get_encoder_estimate(msg_axis_id[2])[0]/gear_ratio*360])
    current = np.array([can_testing_simulation(msg_axis_id[0]), can_testing_simulation(msg_axis_id[1]), can_testing_simulation(msg_axis_id[2])])
    #current = np.array([can_comunication.get_iq(msg_axis_id[0])[1], can_comunication.get_iq(msg_axis_id[1])[1],can_comunication.get_iq(msg_axis_id[1])[1]])

    #caclulate torque and current
    torque = -0.00221823 * current * current + 0.27497038 * current + 0.09294292
    o1 = np.radians(angle[0])
    o2 = np.radians(angle[1])
    o3 = np.radians(angle [2])
    l1 = leg_parameters[0]
    l2 = leg_parameters[1]
    l3 = leg_parameters[2]
    #jacobian calcualted with matlab
    jacobian = np.array([[l2 * np.cos(o1) * np.cos(o2) - l1 * np.sin(o1) + l3 * np.cos(o1) * np.cos(o2) * np.cos(o3) - 1.0 * l3 * np.cos(o1) * np.sin(o2) * np.sin(o3),- l2 * np.sin(o1) * np.sin(o2) - 1.0 * l3 * np.cos(o2) * np.sin(o1) * np.sin(o3) - l3 * np.cos(o3) * np.sin(o1) * np.sin(o2),- l3 * np.cos(o2) * np.sin(o1) * np.sin(o3) - 1.0 * l3 * np.cos(o3) * np.sin(o1) * np.sin(o2)],
                [l1 * np.cos(o1) + 1.0 * l2 * np.cos(o2) * np.sin(o1) + 1.0 * l3 * np.cos(o2) * np.cos(o3) * np.sin(o1) - l3 * np.sin(o1) * np.sin(o2) * np.sin(o3),1.0 * l2 * np.cos(o1) * np.sin(o2) + l3 * np.cos(o1) * np.cos(o2) * np.sin(o3) + 1.0 * l3 * np.cos(o1) * np.cos(o3) * np.sin(o2),1.0 * l3 * np.cos(o1) * np.cos(o2) * np.sin(o3) + l3 * np.cos(o1) * np.cos(o3) * np.sin(o2)],
                [0, - l3 * np.cos(o2 + o3) - l2 * np.cos(o2), -l3 * np.cos(o2 + o3)]])
    return inverse_and_transpose(jacobian).dot(torque)


def contact_detection(leg_id, contact_f):
    if abs(force_calculation(leg_id)[1]) > contact_f:
        return True

###test that leg kinematics move leg to position until force is to high. Then it stops.
leg_parameters = [0.1,0.15,0.15]
print(force_calculation(1,leg_parameters))
# can_comunication.set_closed_loop(5,1)
# can_comunication.set_idle(5,1)
# can_comunication.get_encoder_estimate(5,1)
# can_comunication.move_to(5,0,0,180,1)
# for i in (np.arange(0,90,0.1)):
#    time.sleep(0.5)
#    can_comunication.move_to(5, i, 0, 180, 1)
#    if contact_detection(1,0.1):
#        print("contact")
#        can_comunication.move_to(5, 0, 0, 180, 1)
#        break
###test end
