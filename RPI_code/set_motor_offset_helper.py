import time
import can_comunication
#results below should be copied into the main.py

rang = 10 #1. set motors to configure - one at a tine is recomended- motors hould be manualy held in the desired position
#2. after 5 sec more the motor in the positve define direction.
#3. the console will display the offset inversion value - these should be put into the main.py

current_position = 0
invert_axis = False

 #motor position is captured
current_position = can_comunication.get_encoder_estimate(rang,100)[0]
gear_ratio = 6
angle = current_position/gear_ratio*360
print("the ofset value is:",-angle)
print("now please move in positive direction my any amount")
time.sleep(5)
if can_comunication.get_encoder_estimate(rang,100)[0] < current_position:
    invert_axis = True
print("the inversion value is:",invert_axis )


