import pybullet as p
import time
import pybullet_data
import pathlib
import transformations
import math
import numpy as np


# Start pybullet simulation
p.connect(p.GUI)
# p.connect(p.DIRECT) # don't render

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
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
# robot = p.loadURDF("/urdf/robot.urdf", cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_SELF_COLLISION)
robot = p.loadURDF("/urdf/robot_no_hands.urdf", cubeStartPos, cubeStartOrientation)

humanoid_fix = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], cubeStartPos, [0, 0, 0, 1])
# p.resetBasePositionAndOrientation(robot, cubeStartPos, [0, 0, 0, 1])

jointTypes = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]

LeftUpLeg = 0  # JOINT_SPHERICAL
LeftLeg = 1   # JOINT_REVOLUTE
LeftFoot = 2   # JOINT_REVOLUTE
LeftToeBase = 3   # JOINT_REVOLUTE
RightUpLeg = 4   # JOINT_SPHERICAL
RightLeg = 5   # JOINT_REVOLUTE
RightFoot = 6   # JOINT_REVOLUTE
RightToeBase = 7   # JOINT_REVOLUTE
Spine = 8   # JOINT_FIXED
Spine1 = 9   # JOINT_SPHERICAL
Spine2 = 10   # JOINT_SPHERICAL
LeftShoulder = 11   # JOINT_FIXED
LeftArm = 12   # JOINT_SPHERICAL
LeftForeArm = 13   # JOINT_REVOLUTE
LeftHand = 14   # JOINT_FIXED
LeftHandIndex1 = 15   # JOINT_FIXED
LeftHandIndex2 = 16   # JOINT_FIXED
LeftHandIndex3 = 17   # JOINT_FIXED
LeftHandMiddle1 = 18   # JOINT_FIXED
LeftHandMiddle2 = 19   # JOINT_FIXED
LeftHandMiddle3 = 20   # JOINT_FIXED
LeftHandPinky1 = 21   # JOINT_FIXED
LeftHandPinky2 = 22   # JOINT_FIXED
LeftHandPinky3 = 23   # JOINT_FIXED
LeftHandRing1 = 24   # JOINT_FIXED
LeftHandRing2 = 25   # JOINT_FIXED
LeftHandRing3 = 26   # JOINT_FIXED
LeftHandThumb1 = 27   # JOINT_FIXED
LeftHandThumb2 = 28   # JOINT_FIXED
LeftHandThumb3 = 29   # JOINT_FIXED
Neck = 30   # JOINT_SPHERICAL
Head = 31   # JOINT_REVOLUTE
RightShoulder = 32   # JOINT_FIXED
RightArm = 33   # JOINT_SPHERICAL
RightForeArm = 34   # JOINT_REVOLUTE
RightHand = 35   # JOINT_FIXED
RightHandIndex1 = 36   # JOINT_FIXED
RightHandIndex2 = 37   # JOINT_FIXED
RightHandIndex3 = 38   # JOINT_FIXED
RightHandMiddle1 = 39   # JOINT_FIXED
RightHandMiddle2 = 40   # JOINT_FIXED
RightHandMiddle3 = 41   # JOINT_FIXED
RightHandPinky1 = 42   # JOINT_FIXED
RightHandPinky2 = 43   # JOINT_FIXED
RightHandPinky3 = 44   # JOINT_FIXED
RightHandRing1 = 45   # JOINT_FIXED
RightHandRing2 = 46   # JOINT_FIXED
RightHandRing3 = 47   # JOINT_FIXED
RightHandThumb1 = 48   # JOINT_FIXED
RightHandThumb2 = 49   # JOINT_FIXED
RightHandThumb3 = 50   # JOINT_FIXED

parentLink, legALink, legBLink = -1, 0, 1
enableCollision = 0


# p.setJointMotorControlMultiDof(robot, LeftArm, p.POSITION_CONTROL, [0], targetVelocity=[0, 0, 0], positionGain=0,
#                                velocityGain=1, force=[1000, 1, 1])
#
#
def disable_parent_collision():
    for num in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, num)
        if info[16] != -1:
            p.setCollisionFilterPair(robot, robot, info[0], info[16], 0)


disable_parent_collision()
p.setCollisionFilterPair(robot, robot, Spine2, LeftArm, 0)
p.setCollisionFilterPair(robot, robot, Spine2, RightArm, 0)


for j in range(p.getNumJoints(robot)):
    ji = p.getJointInfo(robot, j)
    targetPosition = [0]
    jointType = ji[2]
    if jointType == p.JOINT_SPHERICAL:
        targetPosition = [0, 0, 0, 1]
        # p.setJointMotorControlMultiDof(robot,
        #                                j,
        #                                p.POSITION_CONTROL,
        #                                targetPosition,
        #                                targetVelocity=[0, 0, 0],
        #                                positionGain=0,
        #                                velocityGain=1,
        #                                force=[1, 1, 1])

    if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
        print(str(ji[1]))
        # p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=0, force=10)
    # print(str(ji))
    # print(str(ji[1]), " = ", j, "# ", jointTypes[ji[2]])


def clench_fists(force=-10.0):
    # LeftHandThumb1 RightHandThumb1
    print("clench force=", force)
    motors = [LeftHandIndex1, LeftHandIndex2, LeftHandIndex3, LeftHandMiddle1, LeftHandMiddle2, LeftHandMiddle3,
              LeftHandPinky1, LeftHandPinky2, LeftHandPinky3, LeftHandRing1, LeftHandRing2, LeftHandRing3,
              LeftHandThumb2, LeftHandThumb3, RightHandIndex1, RightHandIndex2, RightHandIndex3, RightHandMiddle1,
              RightHandMiddle2, RightHandMiddle3, RightHandPinky1, RightHandPinky2, RightHandPinky3, RightHandRing1,
              RightHandRing2, RightHandRing3, RightHandThumb2, RightHandThumb3]
    forces = np.empty(len(motors))
    forces.fill(force)
    print("clench force=", force)
    # p.setJointMotorControlArray(robot, motors, controlMode=p.TORQUE_CONTROL, forces=forces)


print("run simulation")
# step through the simluation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
    # clench_fists()
    # (12, b'link_LeftArm', 2, 29, 24, 0, 0.0, 0.0, -2.0, 2.0, 1000.0, 0.5, b'link_LeftArm', (0.0, 0.0, 0.0), (0.1055, 0.02299, -0.00565), (0.0, 0.0, 0.0, 1.0), 11)
    # print(p.getJointInfo(robot, LeftArm))

for j in range(p.getNumJoints(robot)):
    jointState = p.getJointStateMultiDof(robot, j)
    print("jointStateMultiDof[", j, "].pos=", jointState[0])
    print("jointStateMultiDof[", j, "].vel=", jointState[1])
    print("jointStateMultiDof[", j, "].jointForces=", jointState[3])

p.disconnect()
