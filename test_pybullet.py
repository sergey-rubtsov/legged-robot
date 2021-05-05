import pybullet as p
import time
import pybullet_data
import pathlib
import transformations
import math


# Start pybullet simulation
p.connect(p.GUI)
# p.connect(p.DIRECT) # don't render

# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
# load urdf file path
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# load urdf and set gravity
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# cubeStartOrientation = transformations.quaternion_from_euler(math.pi / 2, 0, 0, 'szyx')
p.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
# boxId = p.loadURDF("/urdf/meshes/stl/robot.urdf", cubeStartPos, cubeStartOrientation)
boxId = p.loadURDF("/urdf/robot.urdf", cubeStartPos, cubeStartOrientation)

# step through the simluation
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    p.getBodyInfo(1)

p.disconnect()