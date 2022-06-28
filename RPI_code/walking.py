import kinematics_legs
import matplotlib.pyplot as plot
import numpy as np
import time as t
from threading import Thread


motor_ofset = [0,0,0,-19,-17,-280,0,0,0,0,0,0]
angle_limit = [180,180,180,300,200,270,180,180,180,180,180,180]
invert_axis = [False,False,False,True,True,True,False,False,False,False,False,False]
leg_parameters = [0.1,0.15,0.15]


def abs_fct(current_val,step_lentgh,accel):
    val = -abs(current_val * accel) + step_lentgh/2 * accel
    return val


def define_curve(leg_id,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight):#neutral hight derined negative
    stance_period = np.pi/step_lentgh
    flight_period = stance_period
    stance_amplitude = 1/stance_max_height
    flight_amplitude = 1/flight_max_heigth

#defining points for stance
    temp_output = -step_lentgh / 2
    imcrement_temp = 1/speed_stance
    output_array_stance_a = []
    while True:
        output_array_stance_a.append(temp_output)
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, acceleration_stance)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_a.append(0)
            break
    temp_output = -step_lentgh / 2
    imcrement_temp = 1/speed_stance
    output_array_stance_b = []
    while True:
        output_array_stance_b.append(temp_output)
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, deceleration_stance)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_b.append(0)
            break
    for i in range(len(output_array_stance_b)):
        output_array_stance_a.append(-output_array_stance_b[i])
    time_stance = np.array(output_array_stance_a)
# defining points for stance end
# defining points for fligt
    temp_output = -step_lentgh / 2
    imcrement_temp = 1/speed_flight
    output_array_stance_a = []
    while True:
        output_array_stance_a.append(temp_output)
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, acceleration_flight)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_a.append(0)
            break
    temp_output = -step_lentgh / 2
    imcrement_temp = 1/speed_flight
    output_array_stance_b = []
    while True:
        output_array_stance_b.append(temp_output)
        imcrement_temp = imcrement_temp + abs_fct(temp_output, step_lentgh, deceleration_flight)
        temp_output = temp_output + imcrement_temp
        if temp_output + imcrement_temp >= 0:
            output_array_stance_b.append(0)
            break
    for i in range(len(output_array_stance_b)):
        output_array_stance_a.append(-output_array_stance_b[i])
    time_flight = np.array(output_array_stance_a)
# defining points for fligt end

    if leg_id == 0 or leg_id == 1:
        stance = - np.sin(time_stance*(stance_period) + np.pi/2) / (stance_amplitude)-neutral_height
        flight = np.sin(time_flight*(flight_period) + np.pi/2) / (flight_amplitude)-neutral_height
    else:
        stance = np.sin(time_stance * (stance_period) + np.pi / 2) / (stance_amplitude) - neutral_height
        flight = -np.sin(time_flight * (flight_period) + np.pi / 2) / (flight_amplitude) - neutral_height
    return [stance,flight,time_stance,time_flight]


def ploting(amplitude_and_time):
    plot.scatter(amplitude_and_time[2], amplitude_and_time[0])
    #plot.scatter(amplitude_and_time[3], amplitude_and_time[1])
    plot.title('Curve')
    plot.xlabel('Time')
    plot.ylabel('Amplitude = sin')
    plot.grid(True, which='both')
    plot.axhline(y=0, color='k')
    plot.show()

ploting(define_curve(1,1,0.1,0.06,-0.2,300,0.01,0.1,300,0.01,0.01))

#walk(1,0.3,0.1,0.06,-0.2,500)

step_lentgh = 0.3
stance_max_height = 0.1
flight_max_heigth = 0.06
neutral_height = -0.2

speed_stance = 300
speed_flight = 300
acceleration_stance = 0.01
deceleration_stance = 0.01
acceleration_flight = 0.01
deceleration_flight = 0.01

def walking_sequence(step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight):
    t1 = Thread(target=walk_stance(1,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t2 = Thread(target=walk_stance(3,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t3 = Thread(target=walk_flight(0,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t4 = Thread(target=walk_flight(2,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5 = Thread(target=walk_stance(0,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t6 = Thread(target=walk_stance(2,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t7 = Thread(target=walk_flight(3,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t8 = Thread(target=walk_flight(1,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed_stance,acceleration_stance,deceleration_stance,speed_flight,acceleration_flight,deceleration_flight))
    t5.start()
    t6.start()
    t7.start()
    t8.start()
    t5.join()
    t6.join()
    t7.join()
    t8.join()

while True:
    walking_sequence(step_lentgh, stance_max_height, flight_max_heigth, neutral_height,speed)

