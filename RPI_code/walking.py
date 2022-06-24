import numpy
import numpy as np
import kinematics_legs
import matplotlib.pyplot as plot
import time as t
import itertools



motor_ofset = [0,0,0,-19,-17,-280,0,0,0,0,0,0]
angle_limit = [180,180,180,300,200,270,180,180,180,180,180,180]
invert_axis = [False,False,False,True,True,True,False,False,False,False,False,False]
leg_parameters = [0.1,0.15,0.15]


def define_curve(leg_id,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed):#neutral hight derined negative
    stance_period = np.pi/step_lentgh
    flight_period = stance_period
    stance_amplitude = 1/stance_max_height
    flight_amplitude = 1/flight_max_heigth

    time_stance = np.arange(-step_lentgh/2, step_lentgh/2+1/speed, 1/speed);
    time_flight = np.arange(step_lentgh/2, -step_lentgh/2-1/speed, -1/speed);
    if leg_id == 0 or leg_id == 1:
        stance = -np.sin(time_stance*(stance_period) + numpy.pi/2) / (stance_amplitude)-neutral_height
        flight = np.sin(time_flight*(flight_period) + numpy.pi/2) / (flight_amplitude)-neutral_height
    else:
        stance = np.sin(time_stance * (stance_period) + numpy.pi / 2) / (stance_amplitude) - neutral_height
        flight = -np.sin(time_flight * (flight_period) + numpy.pi / 2) / (flight_amplitude) - neutral_height
    return [stance,flight,time_stance,time_flight]


def walk (leg_id,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed):
    amplitude_and_time = define_curve(leg_id,step_lentgh,stance_max_height,flight_max_heigth,neutral_height,speed)
    for (stance, flight, time) in zip(amplitude_and_time[0], amplitude_and_time[1], amplitude_and_time[2]):
        kinematics_legs.inverse_kinematics_legs(leg_id, -0.1, stance, time, leg_parameters, motor_ofset, angle_limit, invert_axis)
        t.sleep(1 / speed)
    t.sleep(0.5)
    for (stance, flight, time) in zip(amplitude_and_time[0], amplitude_and_time[1], amplitude_and_time[3]):
        kinematics_legs.inverse_kinematics_legs(leg_id, -0.1, flight, time, leg_parameters, motor_ofset, angle_limit, invert_axis)
        t.sleep(1/speed)




def ploting(amplitude_and_time):
    plot.plot(amplitude_and_time[2], amplitude_and_time[0])
    plot.plot(amplitude_and_time[3], amplitude_and_time[1])
    plot.title('Curve')
    plot.xlabel('Time')
    plot.ylabel('Amplitude = sin')
    plot.grid(True, which='both')
    plot.axhline(y=0, color='k')
    plot.show()

ploting(define_curve(1,0.3,0.1,0.06,-0.2,500))
#walk(1,0.3,0.1,0.06,-0.2,500)