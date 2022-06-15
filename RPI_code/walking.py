#llibaries
import kinematics_legs
import time
import kinematics_spine

def walk_1(leg_parameters,motor_ofset,angle_limit):
    kinematics_legs.inverse_kinematics_legs(1,0,0.4,0,leg_parameters,motor_ofset,angle_limit)#1up
    kinematics_legs.inverse_kinematics_legs(1, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)  # 1down
    time.sleep(0.5)
    kinematics_legs.inverse_kinematics_legs(4, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)#2up
    kinematics_legs.inverse_kinematics_legs(4, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)#2down
    time.sleep(1)
    kinematics_legs.inverse_kinematics_legs(2, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)
    kinematics_legs.inverse_kinematics_legs(2, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)
    time.sleep(0.5)
    kinematics_legs.inverse_kinematics_legs(3, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)
    kinematics_legs.inverse_kinematics_legs(3, 0, 0.4, 0, leg_parameters, motor_ofset, angle_limit)

