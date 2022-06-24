import numpy
import numpy as np
import kinematics_legs
import matplotlib.pyplot as plot
import time
import itertools



motor_ofset = [0,0,0,-16,-12.5,-305,0,0,0,0,0,0]
angle_limit = [180,180,180,180,180,180,180,180,180,180,180,180]
invert_axis = [False,False,False,True,True,True,False,False,False,False,False,False]
leg_parameters = [0.1,0.15,0.15]


def define_curve(step_lentgh,stance_max_height,flight_max_heigth,speed):
    stance_period = np.pi/step_lentgh
    flight_period = stance_period
    stance_amplitude = 1/stance_max_height
    flight_amplitude = 1/flight_max_heigth

    time_stance = np.arange(-step_lentgh/2, step_lentgh/2+1/speed, 1/speed);
    time_flight = np.arange(step_lentgh/2, -step_lentgh/2-1/speed, -1/speed);

    stance = np.sin(time_stance*(stance_period) + numpy.pi/2) / (stance_amplitude)
    flight = -np.sin(time_flight*(flight_period) + numpy.pi/2) / (flight_amplitude)
    return [stance,flight,time_stance,time_flight]


def walk (step_lentgh,stance_max_height,flight_max_heigth,speed):
    amplitude_and_time = define_curve(step_lentgh,stance_max_height,flight_max_heigth,speed)
    for (stance, flight, time) in zip(amplitude_and_time[0], amplitude_and_time[1], amplitude_and_time[2]):
        kinematics_legs.inverse_kinematics_legs(1, -0.1, stance, time, leg_parameters, motor_ofset, angle_limit, invert_axis)
        time.sleep(1 / speed)
        time.sleep(0.5)
    for (stance, flight, time) in zip(amplitude_and_time[0], amplitude_and_time[1], amplitude_and_time[2]):
        kinematics_legs.inverse_kinematics_legs(1, -0.1, flight, time, leg_parameters, motor_ofset, angle_limit, invert_axis)
        time.sleep(1/speed)




def ploting(amplitude_and_time):
    plot.plot(amplitude_and_time[2], amplitude_and_time[0])
    plot.plot(amplitude_and_time[3], amplitude_and_time[1])
    plot.title('Curve')
    plot.xlabel('Time')
    plot.ylabel('Amplitude = sin')
    plot.grid(True, which='both')
    plot.axhline(y=0, color='k')
    plot.show()

ploting(define_curve(0.3,0.12,0.05,500))
walk(0.3,0.15,0.05,100)