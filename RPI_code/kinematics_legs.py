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


