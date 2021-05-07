import pybullet as p
import json
import pybullet_data
import pathlib

useGUI = True
if useGUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

useZUp = False
useYUp = not useZUp
showJointMotorTorques = False

if useYUp:
    p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)

from pdControllerExplicit import PDControllerExplicitMultiDof
from pdControllerStable import PDControllerStableMultiDof

explicitPD = PDControllerExplicitMultiDof(p)
stablePD = PDControllerStableMultiDof(p)

p.resetDebugVisualizerCamera(cameraDistance=7.4,
                             cameraYaw=-94,
                             cameraPitch=-14,
                             cameraTargetPosition=[0.24, -0.02, -0.09])

import pybullet_data

p.setTimeOut(1000000)
useMotionCapture = False
useMotionCaptureReset = False  # not	useMotionCapture
useExplicitPD = True

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=30)
# p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)

# explicit	PD control requires	small	timestep
timeStep = 1. / 600.
# timeStep	=	1./240.

p.setPhysicsEngineParameter(fixedTimeStep=timeStep)

path = str(pathlib.Path(__file__).parent.absolute()) + "/kinematic/humanoid3d_walk.txt"

print("path	=	", path)
with open(path, 'r') as f:
    motion_dict = json.load(f)

print("len motion=", len(motion_dict))
print(motion_dict['Loop'])
numFrames = len(motion_dict['Frames'])
print("#frames = ", numFrames)

frameId = p.addUserDebugParameter("frame", 0, numFrames - 1, 0)

erpId = p.addUserDebugParameter("erp", 0, 1, 0.2)

kpMotorId = p.addUserDebugParameter("kpMotor", 0, 1, .2)
forceMotorId = p.addUserDebugParameter("forceMotor", 0, 2000, 1000)

jointTypes = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]
flags = p.URDF_MAINTAIN_LINK_ORDER + p.URDF_USE_SELF_COLLISION

humanoid4 = p.loadURDF("humanoid/humanoid.urdf",
                       [0, 0, 0],
                       globalScaling=0.25,
                       useFixedBase=False,
                       flags=flags)

humanoid3_fix = p.createConstraint(humanoid4, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                   [0, 0, 0], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(humanoid4, [0, 0, 0], [0, 0, 0, 1])

for j in range(p.getNumJoints(humanoid4)):
    ji = p.getJointInfo(humanoid4, j)
    targetPosition = [0]
    jointType = ji[2]
    if (jointType == p.JOINT_SPHERICAL):
        targetPosition = [0, 0, 0, 1]
        p.setJointMotorControlMultiDof(humanoid4,
                                       j,
                                       p.POSITION_CONTROL,
                                       targetPosition,
                                       targetVelocity=[0, 0, 0],
                                       positionGain=0,
                                       velocityGain=1,
                                       force=[1, 1, 1])

    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        p.setJointMotorControl2(humanoid4, j, p.VELOCITY_CONTROL, targetVelocity=0, force=10)


    # print(ji)
    print("joint[", j, "].type=", jointTypes[ji[2]])
    print("joint[", j, "].name=", ji[1])

chest = 1
neck = 2
rightHip = 3
rightKnee = 4
rightAnkle = 5
rightShoulder = 6
rightElbow = 7
leftHip = 9
leftKnee = 10
leftAnkle = 11
leftShoulder = 12
leftElbow = 13

import time

kpOrg = [
    0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000, 100, 100, 100, 100, 500, 500, 500, 500, 500, 400,
    400, 400, 400, 400, 400, 400, 400, 300, 500, 500, 500, 500, 500, 400, 400, 400, 400, 400, 400,
    400, 400, 300
]
kdOrg = [
    0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 10, 10, 10, 10, 50, 50, 50, 50, 50, 40, 40, 40, 40,
    40, 40, 40, 40, 30, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40, 40, 40, 40, 30
]

p.getCameraImage(320, 200)
frameReal = 0

while (p.isConnected()):
    i = 1 + 1
    # if False:
    #     erp = p.readUserDebugParameter(erpId)
    #     kpMotor = p.readUserDebugParameter(kpMotorId)
    #     maxForce = p.readUserDebugParameter(forceMotorId)
    #     frameReal = p.readUserDebugParameter(frameId)
    # else:
    #     erp = 0.2
    #     kpMotor = 0.2
    #     maxForce = 1000
    #     frameReal = 0
    erp = 0.2
    kpMotor = 0.2
    maxForce = 1000
    frameReal = frameReal + 0.1

    if frameReal >= 37:
        frameReal = 0
    kp = kpMotor

    frame = int(frameReal)
    frameNext = frame + 1
    if frameNext >= numFrames:
        frameNext = frame

    frameFraction = frameReal - frame

    frameData = motion_dict['Frames'][frame]
    frameDataNext = motion_dict['Frames'][frameNext]

    chestRotStart = [frameData[9], frameData[10], frameData[11], frameData[8]]
    chestRotEnd = [frameDataNext[9], frameDataNext[10], frameDataNext[11], frameDataNext[8]]
    chestRot = p.getQuaternionSlerp(chestRotStart, chestRotEnd, frameFraction)

    neckRotStart = [frameData[13], frameData[14], frameData[15], frameData[12]]
    neckRotEnd = [frameDataNext[13], frameDataNext[14], frameDataNext[15], frameDataNext[12]]
    neckRot = p.getQuaternionSlerp(neckRotStart, neckRotEnd, frameFraction)

    rightHipRotStart = [frameData[17], frameData[18], frameData[19], frameData[16]]
    rightHipRotEnd = [frameDataNext[17], frameDataNext[18], frameDataNext[19], frameDataNext[16]]
    rightHipRot = p.getQuaternionSlerp(rightHipRotStart, rightHipRotEnd, frameFraction)

    rightKneeRotStart = [frameData[20]]
    rightKneeRotEnd = [frameDataNext[20]]
    rightKneeRot = [
        rightKneeRotStart[0] + frameFraction * (rightKneeRotEnd[0] - rightKneeRotStart[0])
    ]

    rightAnkleRotStart = [frameData[22], frameData[23], frameData[24], frameData[21]]
    rightAnkleRotEnd = [frameDataNext[22], frameDataNext[23], frameDataNext[24], frameDataNext[21]]
    rightAnkleRot = p.getQuaternionSlerp(rightAnkleRotStart, rightAnkleRotEnd, frameFraction)

    rightShoulderRotStart = [frameData[26], frameData[27], frameData[28], frameData[25]]
    rightShoulderRotEnd = [
        frameDataNext[26], frameDataNext[27], frameDataNext[28], frameDataNext[25]
    ]
    rightShoulderRot = p.getQuaternionSlerp(rightShoulderRotStart, rightShoulderRotEnd,
                                            frameFraction)

    rightElbowRotStart = [frameData[29]]
    rightElbowRotEnd = [frameDataNext[29]]
    rightElbowRot = [
        rightElbowRotStart[0] + frameFraction * (rightElbowRotEnd[0] - rightElbowRotStart[0])
    ]

    leftHipRotStart = [frameData[31], frameData[32], frameData[33], frameData[30]]
    leftHipRotEnd = [frameDataNext[31], frameDataNext[32], frameDataNext[33], frameDataNext[30]]
    leftHipRot = p.getQuaternionSlerp(leftHipRotStart, leftHipRotEnd, frameFraction)

    leftKneeRotStart = [frameData[34]]
    leftKneeRotEnd = [frameDataNext[34]]
    leftKneeRot = [leftKneeRotStart[0] + frameFraction * (leftKneeRotEnd[0] - leftKneeRotStart[0])]

    leftAnkleRotStart = [frameData[36], frameData[37], frameData[38], frameData[35]]
    leftAnkleRotEnd = [frameDataNext[36], frameDataNext[37], frameDataNext[38], frameDataNext[35]]
    leftAnkleRot = p.getQuaternionSlerp(leftAnkleRotStart, leftAnkleRotEnd, frameFraction)

    leftShoulderRotStart = [frameData[40], frameData[41], frameData[42], frameData[39]]
    leftShoulderRotEnd = [frameDataNext[40], frameDataNext[41], frameDataNext[42], frameDataNext[39]]
    leftShoulderRot = p.getQuaternionSlerp(leftShoulderRotStart, leftShoulderRotEnd, frameFraction)
    leftElbowRotStart = [frameData[43]]
    leftElbowRotEnd = [frameDataNext[43]]
    leftElbowRot = [
        leftElbowRotStart[0] + frameFraction * (leftElbowRotEnd[0] - leftElbowRotStart[0])
    ]
    p.resetJointStateMultiDof(humanoid4, chest, chestRot)
    p.resetJointStateMultiDof(humanoid4, neck, neckRot)
    p.resetJointStateMultiDof(humanoid4, rightHip, rightHipRot)
    p.resetJointStateMultiDof(humanoid4, rightKnee, rightKneeRot)
    p.resetJointStateMultiDof(humanoid4, rightAnkle, rightAnkleRot)
    p.resetJointStateMultiDof(humanoid4, rightShoulder, rightShoulderRot)
    p.resetJointStateMultiDof(humanoid4, rightElbow, rightElbowRot)
    p.resetJointStateMultiDof(humanoid4, leftHip, leftHipRot)
    p.resetJointStateMultiDof(humanoid4, leftKnee, leftKneeRot)
    p.resetJointStateMultiDof(humanoid4, leftAnkle, leftAnkleRot)
    p.resetJointStateMultiDof(humanoid4, leftShoulder, leftShoulderRot)
    p.resetJointStateMultiDof(humanoid4, leftElbow, leftElbowRot)
    p.stepSimulation()
    time.sleep(timeStep)
