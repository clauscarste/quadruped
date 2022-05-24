import servo

def inverse_kinematics_spine(x,y,z,ofset):
    angle = [0,0,0]
    servo.set_angle(angle[1],angle[2],angle[3],ofset)