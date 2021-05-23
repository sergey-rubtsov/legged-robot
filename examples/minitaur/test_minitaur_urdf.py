import pybullet as p
import time
import pybullet_data
import pathlib

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.4]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
robot = p.loadURDF("minitaur.urdf", cubeStartPos, cubeStartOrientation)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)