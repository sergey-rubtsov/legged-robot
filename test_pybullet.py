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
cubeStartPos = [0, 0, 1.2]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# cubeStartOrientation = transformations.quaternion_from_euler(math.pi / 2, 0, 0, 'szyx')
p.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
# p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
robot = p.loadURDF("/urdf/robot.urdf", cubeStartPos, cubeStartOrientation,
                   flags=p.URDF_USE_SELF_COLLISION)

humanoid_fix = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                  cubeStartPos, [0, 0, 0, 1])
p.resetBasePositionAndOrientation(robot, cubeStartPos, [0, 0, 0, 1])

jointTypes = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]

for j in range(p.getNumJoints(robot)):
    ji = p.getJointInfo(robot, j)
    targetPosition = [0]
    jointType = ji[2]
    if jointType == p.JOINT_SPHERICAL:
        targetPosition = [0, 0, 0, 1]
        p.setJointMotorControlMultiDof(robot,
                                       j,
                                       p.POSITION_CONTROL,
                                       targetPosition,
                                       targetVelocity=[0, 0, 0],
                                       positionGain=0,
                                       velocityGain=1,
                                       force=[1, 1, 1])

    if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=0, force=10)
    # print(ji)
    print("joint[", j, "]: ", jointTypes[ji[2]], ji[1])

print("run simulation")
# step through the simluation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
