import mujoco_py as mjp
import glfw
import numpy as np

from abr_control.interfaces.mujoco import Mujoco
from abr_control.arms.mujoco_config import MujocoConfig

robot_config = MujocoConfig(xml_file='example.xml', folder='.')
interface = Mujoco(robot_config, dt=0.001)
interface.connect()

try:
    while True:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        interface.send_forces(np.zeros(robot_config.N_JOINTS))

finally:
    interface.disconnect()