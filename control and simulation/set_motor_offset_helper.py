import time
import can_comunication


can_comunication.setall_idle()

# results below should be copied into the main.py

rang = 10
# 1. set motors to configure - one at a tine is recomended- motors hould be manualy held in the desired position
# 2. after 5 sec more the motor in the positve define direction.
# 3. the console will display the offset inversion value - these should be put into the main.py

invert_axis = False

# motor position is captured
for i in [0,1,2,3,4,5,6,7,8,9,10,11,0,1,2,3,4,5,6,7,8,9,10,11,0,1,2,3,4,5,6,7,8,9,10,11]:
    (can_comunication.get_encoder_estimate(i)[0])
for i in [0,1,2,3,4,5,6,7,8,9,10,11]:
    print(can_comunication.get_encoder_estimate(i)[0])
can_comunication.can_get_voltage()
can_comunication.can_get_voltage()
can_comunication.can_get_voltage()
can_comunication.can_get_voltage()
print(can_comunication.can_get_voltage())

current_position = can_comunication.get_encoder_estimate(rang)[0]
gear_ratio = 6
angle = current_position / gear_ratio * 360
print("the ofset value is:", -angle)
print(can_comunication.get_encoder_estimate(rang)[0],
      "this value should not be cloase to 0 or 1, as then user error could ause a problem during setup")
print("now please move in positive direction my any amount")
time.sleep(5)
can_comunication.get_encoder_estimate(rang)[0]
if can_comunication.get_encoder_estimate(rang)[0] < current_position:
    invert_axis = True
print("the inversion value is:", invert_axis)