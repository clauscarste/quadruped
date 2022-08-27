import math
import numpy
import numpy as np
#import can_comunication

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
    motor_angle[0] = math.degrees(motor_angle[
                                      0])   +30 #+10,+100,+140 are necesarry to ajust for the difference in the zero degree position of the motors that also represent the absolute limit in one direction. And the zero degree position of the kinematic model.
    motor_angle[1] = math.degrees(motor_angle[1])   +120
    motor_angle[2] = math.degrees(motor_angle[2])   -160

    print(motor_angle[0]-30, motor_angle[1]-120, motor_angle[2]+160, "this is",leg_id)
    #for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
     #   can_comunication.move_to(i, motor_angle[i-3*leg_id], ofset[i],limit[i],invert_axis[i]) #i-3*leg_id always gives 0,1,2 what equals the first secound and third entry from motor_angle

