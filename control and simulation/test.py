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




< motor name = "torque" joint = "joint0" gear = "1" forcelimited = "true" forcerange = "-10000 10000" / >

kp = 100
kv = 10
motor_number = [0]
for motor_number_i in motor_number:
    set_torque_servo(motor_number_i, 1)
    data.ctrl[motor_number_i] = -kp * (
                data.qpos[motor_number_i] - 2 * can_comunication.position_setpoint[motor_number_i + 1]) - kv * \
                                data.qvel[motor_number_i]  # position control
