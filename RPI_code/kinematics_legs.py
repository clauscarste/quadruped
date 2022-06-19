import math
import numpy
import numpy as np
import time
#import can_comunication
#pip install configparser


motor_ofset = [0,0,0,-16,-12.5,-305,0,0,0,0,0,0]
angle_limit = [180,180,180,180,180,180,180,180,180,180,180,180]
invert_axis = [False,False,False,True,True,True,False,False,False,False,False,False]
leg_parameters = [0.1,0.15,0.15]

def inverse_kinematics_legs(leg_id,x,y,z,leg_parameters,ofset,limit,invert_axis):
    motor_angle = [0,0,0]
    motor_angle[0] = -math.atan2(-y, x) - math.atan2(math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0]), -leg_parameters[0])
    D = (x*x+y*y-leg_parameters[0]*leg_parameters[0]+z*z-leg_parameters[1]*leg_parameters[1]-leg_parameters[2]*leg_parameters[2])/(2*leg_parameters[1]*leg_parameters[2])
    if leg_id == 0 or leg_id == 2:
        motor_angle[2] = math.atan2(-math.sqrt(abs(1-D*D)),D)
    elif leg_id == 1 or leg_id == 3:
        motor_angle[2] = math.atan2(math.sqrt(1-D*D),D)
    motor_angle[1] = math.atan2(z,math.sqrt(x*x+y*y-leg_parameters[0]*leg_parameters[0]) )-math.atan2(leg_parameters[2]*math.sin(motor_angle[2]),leg_parameters[1]+leg_parameters[2]*math.cos(motor_angle[2]))
    motor_angle[0] = math.degrees(motor_angle[0])#+10 #+10,+100,+140 are necesarry to ajust for the difference in the zero degree position of the motors that also represent the absolute limit in one direction. And the zero degree position of the kinematic model.
    motor_angle[1] = math.degrees(motor_angle[1])#+100
    motor_angle[2] = math.degrees(motor_angle[2])#+160

    print(motor_angle[0],motor_angle[1],motor_angle[2])
    #for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
     #   can_comunication.move_to(i, motor_angle[i-3*leg_id], ofset[i],limit[i],invert_axis[i]) #i-3*leg_id always gives 0,1,2 what equals the first secound and third entry from motor_angle



i = 0
while i<10000:
    i= i+0.2
    time.sleep(0.05)
    #print(abs(math.sin(i)*0.2))
    inverse_kinematics_legs(1,-0.1,abs(math.sin(i)*0.25),0,leg_parameters,motor_ofset,angle_limit,invert_axis)