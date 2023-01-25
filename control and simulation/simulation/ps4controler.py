from pyPS4Controller.controller import Controller
import can_comunication
##transf makes that output of controller is in percet so 50% pressing equated to 0.5

def transf(raw):
    temp = (raw+32767)/65534
    # Filter values that are too weak for the motors to move
    if abs(temp) < 0.25:
        return 0
    # Return a value between 0.3 and 1.0
    else:
        return round(temp, 1)

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    def on_left_arrow_press(self):
        #start walking
        print("start walking")
        can_comunication.walk = True
    def on_up_arrow_press(self):
        #jump
        print("jumop")
        can_comunication.jump = True
    def on_up_arrow_press(self):
        #lower
        print("lower")
        can_comunication.lower = True
    def on_circle_press(self):
        #set all to idle state
        print("all axis set to idle")
        can_comunication.set_all_motors_idle = True
    def on_triangle_press(self):
        #set all to closed loop state
        print("all axis set to closed loop")
        can_comunication.set_all_motors_closed_loop = True

    def on_R2_press(self, value):
        # 'value' becomes 0 or a float between 0.3 and 1.0
        value = transf(value)
        print(f"R2 {value}")
        if value > 0.5:
            can_comunication.increase_speed = True
        else:
            can_comunication.decrease_speed = True

    def on_L2_press(self, value):
        # 'value' becomes 0 or a float between 0.3 and 1.0
        value = transf(value)
        print(f"R2 {value}")
        if value > 0.5:
            can_comunication.incease_left = True
        else:
            can_comunication.incease_right = True

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)