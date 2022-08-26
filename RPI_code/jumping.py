import kinematics_legs
import time

def jump(upper,lower,delay,leg_parameters, ofset, limit, invert_axis, leg_config, yawm,
                                            pich, roll, xm, ym, zm, robot_length, robot_with):
    for i in [0,1,2,3]:
        kinematics_legs.inverse_kinematics_legs(i, 0, lower, 0.1, leg_parameters, ofset, limit, invert_axis, leg_config, yawm,
                                                pich, roll, xm, ym, zm, robot_length, robot_with)
    time.sleep(delay)
    for i in [0,1,2,3]:
        kinematics_legs.inverse_kinematics_legs(i, 0, upper, 0.1, leg_parameters, ofset, limit, invert_axis, leg_config, yawm,
                                                pich, roll, xm, ym, zm, robot_length, robot_with)
    time.sleep(delay)
    for i in [0,1,2,3]:
        kinematics_legs.inverse_kinematics_legs(i, 0, lower, 0.1, leg_parameters, ofset, limit, invert_axis, leg_config, yawm,
                                                pich, roll, xm, ym, zm, robot_length, robot_with)