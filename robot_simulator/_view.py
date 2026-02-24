import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("models/robot.xml")
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)
