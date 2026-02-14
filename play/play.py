# import numpy as np
# import time

# import os

# if __name__ == "__main__":

#     script_directory = os.path.dirname(os.path.abspath(__file__))
#     model_path = os.path.join(script_directory, 'simple_model.xml')

#     try:
#         import mujoco as mjc
#         import mujoco.viewer
#         print("successfully imported mujoco as mjc")
#     except Exception as e:
#         print(f"failed to import mujoco do to {e}")
#         raise e from RuntimeError
    
#     model = mjc.MjModel.from_xml_path(model_path)
#     data = mjc.MjData(model)

#     with mujoco.viewer.launch_passive(model, data) as viewer:
#         start = time.time()
#         while viewer.is_running() and time.time() - start < 60:
#             step_start = time.time()
#             if data.time > 5.0 and data.time <10.0:
#                 data.ctrl[0] = 0.5
#             else:
#                 data.ctrl[0] = 0.0
#             mjc.mj_step(model, data)
#             viewer.sync()

#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
#     viewer.close()


import mujoco
import mujoco.viewer
import time
import os

script_directory = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_directory, 'simple_model.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

viewer = mujoco.viewer.launch_passive(model, data)

while data.time < 10.0 and viewer.is_running():

    # ctrl[0] = slew motor (torque)
    # ctrl[1] = luff servo (target angle in radians)
    # ctrl[2] = hoist servo (target distance in meters)

    data.ctrl[1] = 0.5    # hold boom at 0.5 rad (~29°)
    data.ctrl[2] = 1.0    # hold hoist at 1.0m extension

    if data.time < 5.0:
        data.ctrl[0] = 0.5  # spin slew
    else:
        data.ctrl[0] = 0.0  # stop slew

    mujoco.mj_step(model, data)

    if int(data.time / 0.016) != int((data.time - model.opt.timestep) / 0.016):
        viewer.sync()
        time.sleep(0.016)

viewer.close()