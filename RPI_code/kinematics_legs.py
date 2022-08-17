import math
import numpy
import numpy as np
#import can_comunication


def inverse_kinematics_legs(leg_id,x,y,z,leg_parameters,ofset,limit,invert_axis,leg_config):

    motor_angle = [0,0,0]
    motor_angle[0] = -math.atan2(-y, x) - math.atan2(math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0]), -leg_parameters[0])
    D = (x*x+y*y-leg_parameters[0]*leg_parameters[0]+z*z-leg_parameters[1]*leg_parameters[1]-leg_parameters[2]*leg_parameters[2])/(2*leg_parameters[1]*leg_parameters[2])
    if leg_config == 1:
        if leg_id == 0 or leg_id == 2:
            motor_angle[2] = math.atan2(-math.sqrt(abs(1-D*D)),D)
        else:
            motor_angle[2] = math.atan2(math.sqrt(1-D*D),D)
    else:
        motor_angle[2] = math.atan2(-math.sqrt(abs(1 - D * D)), D)
    motor_angle[1] = math.atan2(z,math.sqrt(x*x+y*y-leg_parameters[0]*leg_parameters[0]) )-math.atan2(leg_parameters[2]*math.sin(motor_angle[2]),leg_parameters[1]+leg_parameters[2]*math.cos(motor_angle[2]))
    motor_angle[0] = math.degrees(motor_angle[0])#+30 #+10,+100,+140 are necesarry to ajust for the difference in the zero degree position of the motors that also represent the absolute limit in one direction. And the zero degree position of the kinematic model.
    motor_angle[1] = math.degrees(motor_angle[1])#+120
    motor_angle[2] = math.degrees(motor_angle[2])#+160

    print(motor_angle[0],motor_angle[1],motor_angle[2])
    #for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
        #can_comunication.move_to(i, motor_angle[i-3*leg_id], ofset[i],limit[i],invert_axis[i]) #i-3*leg_id always gives 0,1,2 what equals the first secound and third entry from motor_angle

####Testing
#invert_axis =  [15.8,-27.56,-7,-21.2,-15.6,193,10.3,17.9,-14,-51,24,1.4]
#limit = [300,300,300,300,300,300,300,300,300,300,300]
#ofset = [False,True,True,True,True,True,False,True,True,True,True,True]
#leg_parameters = [0.1,0.15,0.15]

#inverse_kinematics_legs(0,-0.1,-0.20027390523158634,0.09666666666666668,leg_parameters,ofset,limit,invert_axis,1)
####Testing end