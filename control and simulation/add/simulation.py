import math
import mujoco
import os
from mujoco_viewer import MujocoViewer
from spd_utils import (
    computePD,
    populate_show_actuator_forces,
    show_actuator_forces,
)
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


print()
xml_path = 'simulation_files/leg_only_simplided/scene.xml'   # xml file (assumes this is in the same folder as this file)
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
viewer = MujocoViewer(model, data)

# viewer.add_graph_line(line_name="sine_wave", line_data=0.0)
# viewer.add_graph_line(line_name="position_sensor", line_data=0.0)
viewer.add_graph_line(line_name="force", line_data=0.0)
viewer.add_graph_line(line_name="joint_pos_error", line_data=0.0)
x_div = 10
y_div = 10
viewer.set_grid_divisions(x_div=x_div, y_div=y_div, x_axis_time=0.5)
viewer.show_graph_legend()

viewer.callbacks._paused = True
t = 0




while True:




    mujoco.mj_step(model, data)
    viewer.render()

    # viewer.update_graph_line(
    #     line_name="sine_wave",
    #     line_data=target_pos,
    # )
    # viewer.update_graph_line(
    #     line_name="position_sensor",
    #     line_data=data.qpos[0],
    # )
    viewer.update_graph_line(
        line_name="force",
        line_data=1,
    )
    viewer.update_graph_line(
        line_name="joint_pos_error",
        line_data=1,
    )
