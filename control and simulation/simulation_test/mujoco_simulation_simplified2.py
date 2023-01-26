import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
from threading import Thread
import time
import os, sys

##laod interface to current controller ##
from simulation import can_comunication
import main
can_comunication.dictionary()
# create new threads
t2 = Thread(target=main.main_loop)
#t3 = Thread(target=plot.main_loop)
# start the threads
t2.start()
##      ##
xml_path = '../simulation_files/simulation_test/simplified_leg/scene.xml'  # xml file (assumes this is in the same folder as this file)

simend = 30 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0



def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    sim_set_velocity_estimate()
    sim_set_position_estimate()
    #sim_set_gyro()
    sim_set_current_estimate()
    sim_set_measured_force()
    state = sim_get_state()

    #spring-like position servo
    # set_position_servo(1, 10)
    # data.ctrl[1] = np.pi  #position

    #speed control; velocity servo
    # set_velocity_servo(2,100)
    # data.ctrl[2] = 0.5  #velocity

    #position control; position/velocity servo
    #set_position_servo(1, 10)
    #set_velocity_servo(2,1)
    #data.ctrl[1] = 10*can_comunication.position_setpoint[1]

    #torque control;
    #set_torque_servo(0, 1)
    #data.ctrl[0] = -10*(data.qpos[0]-np.pi)  #torque (spring)
    #data.ctrl[0] = -100*(data.qvel[0]-0.5) #speed control]
    #data.ctrl[0] = -100*(data.qpos[0]-can_comunication.position_setpoint[1]) -10*data.qvel[0] #position control
    #data.qpos[1] = can_comunication.position_setpoint[0+1]
    #data.qpos[2] = can_comunication.position_setpoint[0 + 2]


    kp = 3
    kv = 0.1
    motor_number = [0,1]
    for motor_number_i in motor_number:
        if state[motor_number_i] == 1:
            set_torque_servo(motor_number_i, 1)
            data.ctrl[motor_number_i] = -kp * (data.qpos[motor_number_i+7]-(can_comunication.position_setpoint[motor_number_i+1])) - kv * data.qvel[motor_number_i+6]  # position control

def sim_set_position_estimate():
    position_estimate = 0
    can_comunication.position_estimate = position_estimate
def sim_set_velocity_estimate():
    velocity_estimate = 0
    can_comunication.velocity_estimate = velocity_estimate
def sim_set_current_estimate():
    toruqe = 0
    current_estimate = toruqe
    can_comunication.current_estimate = current_estimate
def sim_get_limits(msg_axis_id):
    return [can_comunication.velocity_max_setpoint[msg_axis_id], can_comunication.current_max_setpoint[msg_axis_id]]
def sim_get_state():
    return can_comunication.state
def sim_get_position(msg_axis_id):
    return can_comunication.position_setpoint[msg_axis_id]
def sim_set_gyro():
    quat = np.array([data.qpos[3], data.qpos[4], data.qpos[5],data.qpos[6]])  # always this way if the first joint is the free joint that is the main bosy
    euler = quat2euler(quat)
    can_comunication.gyrodata = [euler,data.sensordata[1]]
def sim_set_measured_force():
    can_comunication.measured_force = data.sensordata[1]



def quat2euler(quat_mujoco):
    #mujocoy quat is constant,x,y,z,
    #scipy quaut is x,y,z,constant
    quat_scipy = np.array([quat_mujoco[3],quat_mujoco[0],quat_mujoco[1],quat_mujoco[2]])

    r = R.from_quat(quat_scipy)
    euler = r.as_euler('xyz', degrees=True)

    return euler

def set_torque_servo(actuator_no, flag):
    if (flag==0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1

def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp

def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv



def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

    if (act == glfw.PRESS and key == glfw.KEY_A):
        print("jumop")
        can_comunication.jump = True
    if (act == glfw.PRESS and key == glfw.KEY_Q):
        print("start walking")
        can_comunication.walk = True
    if (act == glfw.PRESS and key == glfw.KEY_Y):
        print("lower")
        can_comunication.lower = True
    if (act == glfw.PRESS and key == glfw.KEY_DOWN):
        can_comunication.decrease_speed = True
    if (act == glfw.PRESS and key == glfw.KEY_UP):
        can_comunication.increase_speed = True
    if (act == glfw.PRESS and key == glfw.KEY_LEFT):
        can_comunication.incease_left = True
    if (act == glfw.PRESS and key == glfw.KEY_RIGHT):
        can_comunication.incease_right = True
    if (act == glfw.PRESS and key == glfw.KEY_PERIOD):
        print("all axis set to closed loop")
        can_comunication.set_all_motors_closed_loop = True
    if (act == glfw.PRESS and key == glfw.KEY_COMMA):
        print("all axis set to idle")
        can_comunication.set_all_motors_idle = True


def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = -90.68741727466428 ; cam.elevation = -2.8073894766455036 ; cam.distance =  5.457557373462702
cam.lookat =np.array([ 0.0 , 0.0 , 1.0 ])

data.qpos[0] = np.pi/2

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)






while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    print(data.sensordata[0])
    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()







