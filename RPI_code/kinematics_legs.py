#import can_comunication
import math


motor_inital_angle = [0,0,0,0,0,0,0,0,0,0,0,0]
motor_ofset = [0,0,0,0,0,0,0,0,0,0,0,0]
leg_parameters = [0.1,0.6,0.6]

def inverse_kinematics_legs(leg_id,x,y,z,leg_parameters,ofset):
    motor_angle = [0,0,0]

    motor_angle[0] = -math.atan2(-y, x) - math.atan2(math.sqrt(x * x + y * y - leg_parameters[0] * leg_parameters[0]), -leg_parameters[0])
    D = (x*x+y*y-leg_parameters[0]*leg_parameters[0]+z*z-leg_parameters[1]*leg_parameters[1]-leg_parameters[2]*leg_parameters[2])/(2*leg_parameters[1]*leg_parameters[2])
    if leg_id == 1 or leg_id == 2:
        motor_angle[2] = math.atan2(-math.sqrt(abs(1-D*D)),D)
    elif leg_id == 2 or leg_id == 4:
        motor_angle[2] = math.atan2(math.sqrt(1-D*D),D)
    motor_angle[1] = math.atan2(z,math.sqrt(x*x+y*y-leg_parameters[0]*leg_parameters[0]) )-math.atan2(leg_parameters[2]*math.sin(motor_angle[2]),leg_parameters[1]+leg_parameters[2]*math.cos(motor_angle[2]))
    print(math.degrees(motor_angle[0]),math.degrees(motor_angle[1]),math.degrees(motor_angle[2]))
    #for i in range(3*leg_id, 3+(3*leg_id)): #takes 3 of the 12motors that belong to the leg id
     #   can_comunication.move_to(i, math.degrees(motor_angle[i-3*leg_id]), ofset[i])


inverse_kinematics_legs(1,0,0.4,0,leg_parameters,motor_ofset)
