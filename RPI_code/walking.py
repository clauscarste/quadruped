import kinematics_legs
import matplotlib.pyplot as plot
import numpy as np
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
    for i in range(0, len(position[0])):
        kinematics_legs.inverse_kinematics_legs(leg_id, position[1][i], position[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)


def walk_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                robot_length,robot_with):
    position = curve_flight(leg_id, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                            deceleration_flight)
    for i in range(0, len(position[0])):
        kinematics_legs.inverse_kinematics_legs(leg_id, position[1][i], position[0][i], 0.1, leg_parameters, ofset,
                                                limit, invert_axis, leg_config, yaw, pich, roll, xm, ym, zm,
                                                robot_length,robot_with)


def walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height, speed_stance,
                     acceleration_stance, deceleration_stance, speed_flight, acceleration_flight, deceleration_flight,
                     yaw, pich, roll, xm, ym, zm, robot_length,robot_with):
    t1 = Thread(target=walk_stance(1, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                                   deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t2 = Thread(target=walk_stance(3, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                                   deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t3 = Thread(target=walk_flight(0, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                                   deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t4 = Thread(target=walk_flight(2, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                                   deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5 = Thread(target=walk_stance(0, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                                   deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t6 = Thread(target=walk_stance(2, step_lentgh, stance_max_height, neutral_height, speed_stance, acceleration_stance,
                                   deceleration_stance, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t7 = Thread(target=walk_flight(3, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                                   deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t8 = Thread(target=walk_flight(1, step_lentgh, flight_max_heigth, neutral_height, speed_flight, acceleration_flight,
                                   deceleration_flight, leg_parameters, ofset, limit, invert_axis, leg_config, yaw,
                                   pich, roll, xm, ym, zm, robot_length,robot_with))
    t5.start()
    t6.start()
    t7.start()
    t8.start()
    t5.join()
    t6.join()
    t7.join()
    t8.join()


######Testimg
invert_axis = [15.8, -27.56, -7, -21.2, -15.6, 193, 10.3, 17.9, -14, -51, 24, 1.4]
limit = [300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300]
ofset = [False, True, True, True, True, True, False, True, True, True, True, True]
leg_parameters = [0.1, 0.15, 0.15]
leg_config = 1

step_lentgh = 0.2
stance_max_height = 0.1
flight_max_heigth = 0.05
neutral_height = 0.15

speed_stance = 300
speed_flight = 300
acceleration_stance = 0.1
deceleration_stance = 0.001
acceleration_flight = 0.1
deceleration_flight = 0.001

##plot curves
#ploting(curve_stance(1,step_lentgh,stance_max_height,neutral_height,speed_stance,acceleration_stance,deceleration_stance))
#ploting(curve_flight(1,step_lentgh,flight_max_heigth,neutral_height,speed_flight,acceleration_flight,deceleration_flight))

# use walking for one leg
# walk_stance(1,step_lentgh,stance_max_height,neutral_height,speed_stance,acceleration_stance,deceleration_stance,leg_parameters,ofset,limit,invert_axis,leg_config)
# walk_flight(1,step_lentgh,flight_max_heigth,neutral_height,speed_flight,acceleration_flight,deceleration_flight,leg_parameters,ofset,limit,invert_axis,leg_config)

# walking_sequence(step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight)
#.
######Testimg end
