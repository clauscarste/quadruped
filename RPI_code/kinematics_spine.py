#import servo
import math

def inverse_kinematics_spine(omega,theta,d,ofset,max_angle,spine_length,pully_radius,norma_rope_length):
    if theta > 0:
        theta = 0
    elif theta > max_angle:
        theta = max_angle
    #convert degrees to rads
    theta = math.pi/180 * theta

    #calculate length of ropes li
    l = spine_length
    omega_i = [0,0,0]
    li = [0, 0, 0]
    for i in [0,1,2]:
        omega_i[i] = (math.pi/180)*(90+120*(i)-omega)
        li[i] = l - theta * d * math.cos(omega_i[i])

    #clalculate angle that motor has to move as a delta vlaue from the normal length. with negative beein pulling rope inward
    # the ofset must be tuned as such taht this works
    pully_circumference = pully_radius*2*math.pi
    pully_circumference_per_angle = pully_circumference/360
    angle = [0, 0, 0]
    print(li)
    angle[0] = (li[0]-norma_rope_length)/pully_circumference_per_angle
    angle[1] = (li[1]-norma_rope_length)/pully_circumference_per_angle
    angle[2] = (li[2]-norma_rope_length)/pully_circumference_per_angle

    #set motor to that angle
    servo.set_angle(angle[1], angle[2], angle[3], ofset)
    return angle
print(inverse_kinematics_spine(90+120,20,0.07,[0,0,0],20,0.13,0.04,0.13))