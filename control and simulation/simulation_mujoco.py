import mujoco
from simulation.mujoco_viewer import MujocoViewer
from scipy.spatial.transform import Rotation as R

import numpy as np
from threading import Thread
import os
from simulation import force

##laod interface to current controller ##
from simulation import can_comunication
import main
can_comunication.dictionary()
# create new threads
t2 = Thread(target=main.main_loop)
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
viewer.add_graph_line(line_name="measured_force", line_data=0.0)
viewer.add_graph_line(line_name="calc_force", line_data=0.0)
x_div = 10
y_div = 10
viewer.set_grid_divisions(x_div=x_div, y_div=y_div, x_axis_time=0.5)
viewer.show_graph_legend()

viewer.callbacks._paused = True
t = 0

def sim_set_position_estimate():
    can_comunication.position_estimate = data.qpos[7:19]

def sim_set_velocity_estimate():
    can_comunication.velocity_estimate = data.qvel[6:18]
def sim_set_current_estimate():
    toruqe = [data.sensordata[i] for i in [2,5,8,11,14,17,20,23,26,29,32,35]]
    current_estimate = toruqe #must be x5
    can_comunication.current_estimate = current_estimate

def sim_get_limits(msg_axis_id):
    return [can_comunication.velocity_max_setpoint[msg_axis_id], can_comunication.current_max_setpoint[msg_axis_id]]
def sim_get_state():
    return can_comunication.state
def sim_get_position(msg_axis_id):
    return can_comunication.position_setpoint[msg_axis_id]
def sim_set_gyro():
    quat = np.array([data.qpos[3], data.qpos[4], data.qpos[5],data.qpos[6]])  # always this way if the first joint is the free joint that is the main body
    euler = quat2euler(quat)
    can_comunication.gyrodata[0:3] = euler
    can_comunication.gyrodata[3:9] = data.sensordata[36:42]
def sim_set_measured_force():
    can_comunication.measured_force = data.sensordata[42:45]

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
    pos_control = False
    sim_set_velocity_estimate()
    sim_set_position_estimate()
    sim_set_gyro()
    #print(data.sensordata[100])51
    #print(can_comunication.measured_force)
    #print(can_comunication.position_estimate[0:3])
    can_comunication.force_estimate = force.force_calculation()
    sim_set_current_estimate()
    sim_set_measured_force()
    state = sim_get_state()
    kp = can_comunication.kpkv[0]
    kv = can_comunication.kpkv[1]
    motor_number = [0,1,2,3,4,5,6,7,8,9,10,11]
    for motor_number_i in motor_number:
        if can_comunication.state[motor_number_i] == 1:
            set_torque_servo(motor_number_i, 1)
            if pos_control == True:
                data.ctrl[motor_number_i] = -kp * (data.qpos[motor_number_i+7]-(can_comunication.position_setpoint[motor_number_i])) - kv * data.qvel[motor_number_i+6]  # position control
            else:
                data.ctrl[motor_number_i] = can_comunication.torque_setpoint[motor_number_i]
                #data.ctrl[motor_number_i] = 10

    mujoco.mj_step(model, data)
    viewer.render()


    axis = can_comunication.swaitchgraph
    showforce = True #cange to false to show torque and position
    if showforce == False:
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
        #for comparing measured force with calculted froce from torque
    else:
        viewer.update_graph_line(
            line_name="torque",
            line_data=None,
        )
        viewer.update_graph_line(
            line_name="position",
            line_data=None,
        )
        viewer.update_graph_line(
            line_name="comanded_position",
            line_data=None,
        )
        viewer.update_graph_line(
            line_name="measured_force",
            line_data=can_comunication.measured_force[2],
        )
        viewer.update_graph_line(
            line_name="calc_force",
            line_data=can_comunication.force_estimate[2],
        )