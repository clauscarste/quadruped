#for real world testing
#from realworld import can_comunication

#for simulation
from simulation import can_comunication

import math
import numpy
import numpy as np


# add roll_pich_yaw controll and ajust for the diferent orientation of the leg cordinate frames
def yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z):
    if leg_id == 0:
        y = -y
        #T0
        T0 = np.array([[-np.sin(yaw), -np.cos(yaw) * np.sin(pich), np.cos(pich) * np.cos(yaw),
                        (robot_with * np.sin(yaw)) / 2 + zm * np.sin(yaw) - (
                                    robot_length * np.cos(pich) * np.cos(yaw)) / 2 + xm * np.cos(
                            pich) * np.cos(yaw) - ym * np.cos(yaw) * np.sin(pich)],
                       [np.cos(yaw) * np.sin(roll), np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw),
                        np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw),
                        xm * (np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw)) - (
                                robot_length * (
                                    np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw))) / 2 + ym * (
                                np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw)) - (
                                robot_with * np.cos(yaw) * np.sin(roll)) / 2 - zm * np.cos(yaw) * np.sin(roll)],
                       [-np.cos(roll) * np.cos(yaw),
                        np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw),
                        np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw),
                        xm * (np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw)) - (
                                robot_length * (
                                    np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw))) / 2 + ym * (
                                np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw)) + (
                                robot_with * np.cos(roll) * np.cos(yaw)) / 2 + zm * np.cos(roll) * np.cos(yaw)],
                       [0, 0, 0, 1]])
        return np.matmul(np.array([x,y,z,0]),T0)
    elif leg_id == 1:
        y = -y
        #T1
        T1 = np.array([[-np.sin(yaw), -np.cos(yaw) * np.sin(pich), np.cos(pich) * np.cos(yaw),
                        (robot_with * np.sin(yaw)) / 2 + zm * np.sin(yaw) + (
                                    robot_length * np.cos(pich) * np.cos(yaw)) / 2 + xm * np.cos(pich) * np.cos(
                            yaw) - ym * np.cos(yaw) * np.sin(pich)],
                       [np.cos(yaw) * np.sin(roll), np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw),
                        np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw), (robot_length * (
                                   np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw))) / 2 + xm * (
                                    np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw)) + ym * (
                                    np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw)) - (
                                    robot_with * np.cos(yaw) * np.sin(roll)) / 2 - zm * np.cos(yaw) * np.sin(roll)],
                       [-np.cos(roll) * np.cos(yaw),
                        np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw),
                        np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw), (robot_length * (
                                   np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw))) / 2 + xm * (
                                    np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw)) + ym * (
                                    np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw)) + (
                                    robot_with * np.cos(roll) * np.cos(yaw)) / 2 + zm * np.cos(roll) * np.cos(yaw)],
                       [0, 0, 0, 1]])
        return np.matmul(np.array([x,y,z,0]),T1)

    elif leg_id == 2:
        y = -y
        z = -z
        x = -x
    #T2
        T2 = np.array([[np.sin(yaw), -np.cos(yaw) * np.sin(pich), -np.cos(pich) * np.cos(yaw),
                        zm * np.sin(yaw) - (robot_with * np.sin(yaw)) / 2 + (
                                    robot_length * np.cos(pich) * np.cos(yaw)) / 2 + xm * np.cos(
                            pich) * np.cos(yaw) - ym * np.cos(yaw) * np.sin(pich)],
                       [-np.cos(yaw) * np.sin(roll),
                        np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw),
                        -np.cos(roll) * np.sin(pich) - np.cos(pich) * np.sin(roll) * np.sin(yaw),
                        (robot_length * (
                                    np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw))) / 2 + xm * (
                                np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw)) + ym * (
                                np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw)) + (
                                robot_with * np.cos(yaw) * np.sin(roll)) / 2 - zm * np.cos(yaw) * np.sin(roll)],
                       [np.cos(roll) * np.cos(yaw), np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw),
                        np.cos(pich) * np.cos(roll) * np.sin(yaw) - np.sin(pich) * np.sin(roll),
                        (robot_length * (
                                    np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw))) / 2 + xm * (
                                np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw)) + ym * (
                                np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw)) - (
                                robot_with * np.cos(roll) * np.cos(yaw)) / 2 + zm * np.cos(roll) * np.cos(yaw)],
                       [0, 0, 0, 1]])
        return np.matmul(np.array([x,y,z,0]),T2)

    elif leg_id == 3:
        y = -y
        z = -z
        x = -x
        #T3
        T3 = np.array([[np.sin(yaw), -np.cos(yaw) * np.sin(pich), -np.cos(pich) * np.cos(yaw),
                        zm * np.sin(yaw) - (robot_with * np.sin(yaw)) / 2 - (
                                    robot_length * np.cos(pich) * np.cos(yaw)) / 2 + xm * np.cos(
                            pich) * np.cos(yaw) - ym * np.cos(yaw) * np.sin(pich)],
                       [-np.cos(yaw) * np.sin(roll),
                        np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw),
                        -np.cos(roll) * np.sin(pich) - np.cos(pich) * np.sin(roll) * np.sin(yaw),
                        xm * (np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw)) - (
                                robot_length * (
                                    np.cos(roll) * np.sin(pich) + np.cos(pich) * np.sin(roll) * np.sin(yaw))) / 2 + ym * (
                                np.cos(pich) * np.cos(roll) - np.sin(pich) * np.sin(roll) * np.sin(yaw)) + (
                                robot_with * np.cos(yaw) * np.sin(roll)) / 2 - zm * np.cos(yaw) * np.sin(roll)],
                       [np.cos(roll) * np.cos(yaw), np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw),
                        np.cos(pich) * np.cos(roll) * np.sin(yaw) - np.sin(pich) * np.sin(roll),
                        xm * (np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw)) - (
                                robot_length * (
                                    np.sin(pich) * np.sin(roll) - np.cos(pich) * np.cos(roll) * np.sin(yaw))) / 2 + ym * (
                                np.cos(pich) * np.sin(roll) + np.cos(roll) * np.sin(pich) * np.sin(yaw)) - (
                                robot_with * np.cos(roll) * np.cos(yaw)) / 2 + zm * np.cos(roll) * np.cos(yaw)],
                       [0, 0, 0, 1]])
        return np.matmul(np.array([x,y,z,0]),T3)



# condinates always seen fomr the cnter cordinate frame
def inverse_kinematics_legs(leg_id, x, y, z, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm,
                            ym, zm, robot_length,robot_with):
    # ajust for pich jaw roll and fram orientation
    converted_condintes = yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z)
    x = converted_condintes[0]
    y = converted_condintes[1]
    z = converted_condintes[2]
    motor_angle = [0, 0, 0]
    motor_angle[0] = -math.atan2(-y, x) - math.atan2(math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0]),
                                                     -leg_parameters[0])
    D = (x * x + y * y - leg_parameters[0] * leg_parameters[0] + z * z - leg_parameters[1] * leg_parameters[1] -
         leg_parameters[2] * leg_parameters[2]) / (2 * leg_parameters[1] * leg_parameters[2])
    if leg_config == 1:
        if leg_id == 0 or leg_id == 2:
            motor_angle[2] = math.atan2(-math.sqrt(abs(1 - D * D)), D)
        else:
            motor_angle[2] = math.atan2(math.sqrt(1 - D * D), D)
    else:
        motor_angle[2] = math.atan2(math.sqrt(1 - D * D), D)
    motor_angle[1] = math.atan2(z, math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0])) - math.atan2(
        leg_parameters[2] * math.sin(motor_angle[2]), leg_parameters[1] + leg_parameters[2] * math.cos(motor_angle[2]))

    ##fore detection part
    if contact_detection(leg_id, 10, leg_parameters, motor_angle[0], motor_angle[1], motor_angle[2]):
        print("force exeeded")
        return

    motor_angle[0] = math.degrees(motor_angle[                                                                             #uncomment
                                      0])#   +30 #+10,+100,+140 are necesarry to ajust for the difference in the zero degree position of the motors that also represent the absolute limit in one direction. And the zero degree position of the kinematic model.
    motor_angle[1] = math.degrees(motor_angle[1])#   +120
    motor_angle[2] = math.degrees(motor_angle[2])#   -160

    #print(motor_angle[0]-30, motor_angle[1]-120, motor_angle[2]+160, "this is",leg_id)
    #if leg_id == 1:
        #print(motor_angle[0], motor_angle[1], motor_angle[2])
        #can_comunication.printit(motor_angle[0], motor_angle[1], motor_angle[2])
    for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
        can_comunication.move_to(i, motor_angle[i-3*leg_id], ofset[i],limit[i],invert_axis[i]) #i-3*leg_id always gives 0,1,2 what equals the first secound and third entry from motor_angle





## added the force calculation to the inverse kinematics in order to have the command not sent if the limit is exeeded, this helps with the response


def inverse_and_transpose(m):
    return np.transpose(np.linalg.inv(m))

def force_calculation(leg_id, leg_parameters,angle1,angle2,angle3):
    # convert leg id to msg axis
    msg_axis_id = [leg_id * 3, leg_id * 3 + 1, leg_id * 3 + 2]
    # get current and ajust for roation direction
    array_temp = 0
    current = np.array([0.0,0.0,0.0])
    for i in msg_axis_id:
        temp =can_comunication.get_iq(i)[1]
        if i in [1,2,3,4,5,9]:
            temp = -temp
        current[array_temp] = temp
        array_temp = array_temp+1
    # caclulate torque and current
    torque = -0.00221823 * current * current + 0.27497038 * current + 0.09294292
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


def contact_detection(leg_id, contact_f,leg_parameters,angle1,angle2,angle3):
    if abs(force_calculation(leg_id,leg_parameters,angle1,angle2,angle3)[1]) > contact_f:
        return True
