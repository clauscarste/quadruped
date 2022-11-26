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
# start the threads
t2.start()
##      ##

xml_path = 'simulation_files/leg_only/scene.xml' #xml file (assumes this is in the same folder as this file)
simend = 20 #simulation time
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
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

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


#setter and getter from interface dictionary
def sim_set_position_estimate():
    position_estimate = data.sensordata[0:3]
    can_comunication.position_estimate = position_estimate
def sim_set_velocity_estimate():
    velocity_estimate = data.sensordata[3:6]
    can_comunication.velocity_estimate = velocity_estimate
def sim_set_current_estimate():
    toruqe = data.sensordata[6:9]
    current_estimate = 61.9797 - 4.5081*10**-6* np.sqrt(191083462427677 - 22182300000000*toruqe)
    can_comunication.current_estimate = current_estimate
def sim_get_limits(msg_axis_id):
    return [can_comunication.velocity_max_setpoint[msg_axis_id], can_comunication.current_max_setpoint[msg_axis_id]]
def sim_get_state(msg_axis_id):
    return can_comunication.state[msg_axis_id]
def sim_get_position(msg_axis_id):
    return can_comunication.position_setpoint[msg_axis_id]
def sim_set_gyro():
    quat = np.array([data.qpos[3], data.qpos[4], data.qpos[5],data.qpos[6]])  # always this way if the first joint is the free joint that is the main bosy
    euler = quat2euler(quat)
    can_comunication.gyrodata = [euler,data.sensordata[3:6]]
def sim_set_measured_force(force_data):
    can_comunication.measured_force = data.sensordata[3:6]

def quat2euler(quat_mujoco):
    #mujocoy quat is constant,x,y,z,
    #scipy quaut is x,y,z,constant
    quat_scipy = np.array([quat_mujoco[3],quat_mujoco[0],quat_mujoco[1],quat_mujoco[2]])

    r = R.from_quat(quat_scipy)
    euler = r.as_euler('xyz', degrees=True)

    return euler








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
#initialize the controller here. This function is called once, in the beginning
cam.azimuth = 154.58669433593755 ; cam.elevation =  -40.60546875000001 ; cam.distance =  1.04038754800176
cam.lookat =np.array([ -0.042281823710760245 , -0.08899197085290025 , 0.14095040418132013 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

N = 500
q0_start = 0;
q1_start = 0;



#initialize
data.qpos[0] = q0_start;
data.qpos[1] = q1_start
i = 0;
time = 0
dt = 0.001;
poi = np.linspace(0,3,N)

while not glfw.window_should_close(window):
    time_prev = time

    while (time - time_prev < 1.0/60.0):
        data.qpos[0] = 0#can_comunication.position_setpoint[0];
        data.qpos[1] = 0#can_comunication.position_setpoint[1];
        data.qpos[2] = poi[i]#can_comunication.position_setpoint[2];


        print(data.sensordata[6:9], "data")
        #sim_set_position_estimate()
        #print(can_comunication.position_estimate, "can")
        #data.qpos[2] = can_comunication.position_setpoint[1];
        #data.qpos[3] = can_comunication.position_setpoint[2];
        #print(data.qpos[0])
        mj.mj_forward(model,data)
        time +=dt
        # mj.mj_step(model, data)

    i +=1

    #print(data.site_xpos[0])
    
    if (i>=N):
        break;
    # if (data.time>=simend):
    #     break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

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
