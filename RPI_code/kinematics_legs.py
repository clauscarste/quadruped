import math
import numpy
import numpy as np


# import can_comunication

# add roll_pich_yaw controll and ajust for the diferent orientation of the leg cordinate frames
def yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z):
    if leg_id == 0:
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
        print(T0)
        return np.matmul(np.array([x,y,z,0]),T0)
    elif leg_id == 1:
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
        motor_angle[2] = math.atan2(-math.sqrt(abs(1 - D * D)), D)
    motor_angle[1] = math.atan2(z, math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0])) - math.atan2(
        leg_parameters[2] * math.sin(motor_angle[2]), leg_parameters[1] + leg_parameters[2] * math.cos(motor_angle[2]))
    motor_angle[0] = math.degrees(motor_angle[
                                      0])  # +30 #+10,+100,+140 are necesarry to ajust for the difference in the zero degree position of the motors that also represent the absolute limit in one direction. And the zero degree position of the kinematic model.
    motor_angle[1] = math.degrees(motor_angle[1])  # +120
    motor_angle[2] = math.degrees(motor_angle[2])  # +160

    print(motor_angle[0], motor_angle[1], motor_angle[2])
    # for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
    # can_comunication.move_to(i, motor_angle[i-3*leg_id], ofset[i],limit[i],invert_axis[i]) #i-3*leg_id always gives 0,1,2 what equals the first secound and third entry from motor_angle

####Testing
# invert_axis =  [15.8,-27.56,-7,-21.2,-15.6,193,10.3,17.9,-14,-51,24,1.4]
# limit = [300,300,300,300,300,300,300,300,300,300,300]
# ofset = [False,True,True,True,True,True,False,True,True,True,True,True]
# leg_parameters = [0.1,0.15,0.15]
leg_id= 1
x = 1
y = 2
z = 0
yaw = 0
pich= 20
roll= 0
xm = 0
ym = 0
zm = 0
robot_length = 10
robot_with = 4
print(yaw_pich_roll(yaw, pich, roll, xm, ym, zm, robot_length,robot_with, leg_id, x, y, z))
# inverse_kinematics_legs(0,-0.1,-0.20027390523158634,0.09666666666666668,leg_parameters,ofset,limit,invert_axis,1,,yaw,pich,roll,xm,ym,zm,robot_length,robot_with)
####Testing end
