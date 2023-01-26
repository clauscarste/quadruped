import math
import mujoco
import os
from simulation.mujoco_viewer import MujocoViewer

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




xml_path = 'simulation_files/full_quadruped_simulation_servo/scene.xml'   # xml file (assumes this is in the same folder as this file)
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
viewer = MujocoViewer(model, data)




viewer.add_graph_line(line_name="torque", line_data=0.0)
viewer.add_graph_line(line_name="position", line_data=0.0)
viewer.add_graph_line(line_name="comanded_position", line_data=0.0)
x_div = 10
y_div = 10
viewer.set_grid_divisions(x_div=x_div, y_div=y_div, x_axis_time=0.5)
viewer.show_graph_legend()

viewer.callbacks._paused = True
t = 0

def sim_set_position_estimate():
    position_estimate = aa = [data.sensordata[i] for i in [0, 1,2,5,16,17,30,31,32]]
    can_comunication.position_estimate = position_estimate
def sim_set_velocity_estimate():
    velocity_estimate = [data.sensordata[i] for i in [3,4,5,18,19,20,33,34,35]]
    can_comunication.velocity_estimate = velocity_estimate
def sim_set_current_estimate():
    toruqe = [data.sensordata[i] for i in [8,11,14,23,26,29,38,41,44]] #53,56,59
    current_estimate = toruqe #must be x5
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

while True:
    sim_set_velocity_estimate()
    sim_set_position_estimate()
    #sim_set_gyro()
    sim_set_current_estimate()
    sim_set_measured_force()
    state = sim_get_state()
    kp = can_comunication.kpkv[0]
    kv = can_comunication.kpkv[1]
    motor_number = [0,1,2,3,4,5,6,7,8,9,10,11]
    for motor_number_i in motor_number:
        if can_comunication.state[motor_number_i] == 1:
            set_torque_servo(motor_number_i, 1)
            data.ctrl[motor_number_i] = -kp * (data.qpos[motor_number_i+7]-(can_comunication.position_setpoint[motor_number_i])) - kv * data.qvel[motor_number_i+6]  # position control

    mujoco.mj_step(model, data)
    viewer.render()


    axis = can_comunication.swaitchgraph
    if can_comunication.showtorque == True:
        viewer.update_graph_line(
            line_name="torque",
            line_data=can_comunication.current_estimate[axis],
        )
    else:
        viewer.update_graph_line(
            line_name="torque",
            line_data=None,
        )
    viewer.update_graph_line(
        line_name="position",
        line_data=can_comunication.position_estimate[axis],
    )
    viewer.update_graph_line(
        line_name="comanded_position",
        line_data=can_comunication.position_setpoint[axis],
    )