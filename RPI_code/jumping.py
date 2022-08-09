import kinematics_legs
import time

def jump(leg_parameters, ofset, limit, invert_axis):
    kinematics_legs.inverse_kinematics_legs(1,1,1,1,leg_parameters,ofset,limit,invert_axis)
    kinematics_legs.inverse_kinematics_legs(2, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)
    kinematics_legs.inverse_kinematics_legs(3, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)
    kinematics_legs.inverse_kinematics_legs(4, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)

    time.sleep(0.5)
    kinematics_legs.inverse_kinematics_legs(1, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)
    kinematics_legs.inverse_kinematics_legs(2, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)
    kinematics_legs.inverse_kinematics_legs(3, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)
    kinematics_legs.inverse_kinematics_legs(4, 1, 1, 1, leg_parameters, ofset, limit, invert_axis)

jump()