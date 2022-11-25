import time
#import can_comunication                                                                                                 ##uncomment
import kinematics_legs
import matplotlib.pyplot as plot
import numpy as np
#import can_thread
import time as t
from threading import Thread



# for walking
# 1: define curve in the "curve_stance" and "curve_flight" fct for stance and flight, that give the y/z
# position of the foot at indicidual points for these two movments the movement is defined by many parameters such as
# acceleration.
# 2: send the position information from step 1 to the "kinematics_legs" fct using the "walk_stance" and
# "walk_flight" fct to have the kinematic solution calcualted, to then sned that information to the motors
# 3: the two steps above are exectued in a certain order for all legs using threading

def abs_fct(current_val, step_lentgh, accel):
    val = -abs(current_val * accel) + step_lentgh / 2 * accel
    return val


def curve_stance(leg_id, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                 deceleration_stance):
    stance_max_height = stance_max_height / 2
    stance_period = np.pi / (step_lentgh / 2)
    stance_amplitude = 1 / stance_max_height

    # defining points for stance
    temp_output = -step_lentgh / 2
    imcrement_temp = 1 / speed_stance
    output_array_stance_a = []
    # first half of the stane curve
    while True:
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, acceleration_stance)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_a.append(0)
            break
        output_array_stance_a.append(temp_output)
    # other half of the stane curve
    temp_output = -step_lentgh / 2
    imcrement_temp = 1 / speed_stance
    output_array_stance_b = []
    while True:
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, deceleration_stance)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_b.append(0)
            break
        output_array_stance_b.append(temp_output)
        # adding the two curves together
    for i in range(len(output_array_stance_b)):
        output_array_stance_a.append(-output_array_stance_b[len(output_array_stance_b) - i - 1])
    time_stance = np.array(output_array_stance_a)
    # defining points for stance end

    # caclulating sin curves at the given time
    stance = - np.sin(time_stance * (stance_period) + np.pi / 2) / (stance_amplitude) - (
            neutral_height + stance_max_height)
    return [stance, time_stance]


def curve_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                 deceleration_flight):
    tempp = deceleration_flight
    deceleration_flight = acceleration_flight
    acceleration_flight = tempp
    flight_max_heigth = flight_max_heigth / 2
    flight_period = np.pi / (step_lentgh / 2)
    flight_amplitude = 1 / flight_max_heigth
    # defining points for flight
    temp_output = -step_lentgh / 2
    imcrement_temp = 1 / speed_flight
    output_array_flight_a = []
    # first half of the flight curve
    while True:
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, acceleration_flight)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_flight_a.append(0)
            break
        output_array_flight_a.append(temp_output)
    # other half of the flight curve
    temp_output = -step_lentgh / 2
    imcrement_temp = 1 / speed_flight
    output_array_flight_b = []
    while True:
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, deceleration_flight)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_flight_b.append(0)
            break
        output_array_flight_b.append(temp_output)
        # adding the two curves together

    for i in range(len(output_array_flight_b)):
        output_array_flight_a.append(-output_array_flight_b[len(output_array_flight_b) - i - 1])
    time_flight = np.array(output_array_flight_a)
    # defining points for fligt end

    # caclulating sin curves at the given time
    flight = np.sin(time_flight * (flight_period) + np.pi / 2) / (flight_amplitude) - (neutral_height - flight_max_heigth)
    #reverting the arrays so the movement is right - start point of flight is end point of stance
    flight = np.flip(flight)
    time_flight = np.flip(time_flight)
    return [flight, time_flight]


# Plot the curve
def ploting(amplitude_and_time):
    plot.scatter(amplitude_and_time[1], amplitude_and_time[0])
    plot.title('Curve')
    plot.xlabel('Y')
    plot.ylabel('Z')
    plot.grid(True, which='both')
    plot.axhline(y=0, color='k')
    plot.show()


def walk_stance(leg_id, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                robot_length,robot_with):
    position = curve_stance(leg_id, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                            deceleration_stance)
    for i in range(len(position[0])-1,0, -1):
        time.sleep(0.05)
        kinematics_legs.inverse_kinematics_legs(leg_id, position[1][i], position[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)


def walk_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                robot_length,robot_with):
    position = curve_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                            deceleration_flight)
    for i in range(len(position[0])-1,0, -1):
        time.sleep(0.05)
        kinematics_legs.inverse_kinematics_legs(leg_id, position[1][i], position[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)

def step(stance1,stance2,flight1,flight2,flight_max_heigth,acceleration_flight,deceleration_flight,
         speed_flight, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
         deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
         robot_length,robot_with):
    position1 = curve_stance(stance1, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance,deceleration_stance)
    position2 = curve_stance(stance2, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance,deceleration_stance)
    position3 = curve_stance(flight1, step_lentgh, flight_max_heigth, neutral_height, speed_flight,
                             acceleration_flight,deceleration_flight)
    position4 = curve_stance(flight2, step_lentgh, flight_max_heigth, neutral_height, speed_flight,
                             acceleration_flight,deceleration_flight)
    for i in range(len(position1[0])-2,0, -1):
    #    for a in [4,10]:
    #        if can_thread.loop_state[a] != 0x8:                                                                        #uncomment
    #            can_comunication.set_closed_loop(a,2)
        time.sleep(0.5)
        kinematics_legs.inverse_kinematics_legs(stance1, position1[1][i], position1[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(stance2, position2[1][i], position2[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(flight1, position3[1][i], position3[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(flight2, position4[1][i], position4[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)


def walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
                     acceleration_stance, deceleration_stance, speed_flight, acceleration_flight, deceleration_flight,
                     yaw, pich, roll, xm, ym, zm, robot_length,robot_with,leg_parameters, ofset, limit,
                     invert_axis, leg_config):
    for i in range(0,2):
        step(1, 3, 0, 2,flight_max_heigth,acceleration_flight,deceleration_flight,
             speed_flight, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
            deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
            robot_length,robot_with)
        step(0, 2, 1, 3,flight_max_heigth,acceleration_flight,deceleration_flight,
             speed_flight, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
            deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
            robot_length,robot_with)



def one_step_at_a_time(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
                     acceleration_stance, deceleration_stance, speed_flight, acceleration_flight, deceleration_flight,
                     yaw, pich, roll, xm, ym, zm, robot_length,robot_with,leg_parameters, ofset, limit,
                     invert_axis, leg_config):
    #for a in [4, 10]:                                                                                                  #uncomment
    #    if can_thread.loop_state[a] != 0x8:
    #        can_comunication.set_closed_loop(a, 2)
    walk_flight(3, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                robot_length, robot_with)
    time.sleep(0.5)
    walk_flight(1, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                robot_length, robot_with)
    time.sleep(2)
    stance1 = 0
    stance2 = 1
    flight1 = 2
    flight2 = 3
    position1 = curve_stance(stance1, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance,deceleration_stance)
    position2 = curve_stance(stance2, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance,deceleration_stance)
    position3 = curve_stance(stance1, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance, deceleration_stance)
    position4 = curve_stance(stance2, step_lentgh, stance_max_height, neutral_height, speed_stance,
                             acceleration_stance, deceleration_stance)
    for i in range(0, len(position1[0])-2, -1):
#        for a in [4, 10]:
#            if can_thread.loop_state[a] != 0x8:                                                                        #uncomment
#                can_comunication.set_closed_loop(a, 2)
        time.sleep(0.05)
        kinematics_legs.inverse_kinematics_legs(stance1, position1[1][i], position1[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(stance2, position2[1][i], position2[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(flight1, position3[1][i], position3[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
        kinematics_legs.inverse_kinematics_legs(flight2, position4[1][i], position4[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)
    time.sleep(2)
    walk_flight(0, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                    deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym,
                    zm,robot_length, robot_with)
    time.sleep(0.5)
    walk_flight(2, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                    deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym,
                    zm,robot_length, robot_with)

