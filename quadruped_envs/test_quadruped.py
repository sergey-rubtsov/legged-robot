"""
        a   b   c   d
hips    0   3   6   9
uleg    1   4   7   10
lleg    2   5   8   11

laikago:
hips    3   0   9   6
uleg    4   1  10   7
lleg    5   2  11   8

"""

import pybullet as p
import time
import pybullet_data
import pathlib
import transformations
import math
import numpy as np
from pybullet_utils import pd_controller_stable
from pybullet_envs.deep_mimic.env import motion_capture_data
from pybullet_envs.deep_mimic.env import quadrupedPoseInterpolator

# Start pybullet simulation
from quadruped_envs.open_dynamic_quadruped import OpenDynamicQuadruped

p.connect(p.GUI)
# p.connect(p.DIRECT) # don't render

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
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
cubeStartPos = [0, 0, 0.3]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])
# cubeStartOrientation = transformations.quaternion_from_euler(math.pi / 2, 0, 0, 'szyx')
p.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
env = OpenDynamicQuadruped(pybullet_client=p)
p.addUserDebugParameter(paramName='111',
                                                 rangeMin=0,
                                                 rangeMax=10,
                                                 startValue=5)
mocapData = motion_capture_data.MotionCaptureData()

motionPath = pybullet_data.getDataPath() + "/data/motions/laikago_walk.txt"

mocapData.Load(motionPath)
print("mocapData.NumFrames=", mocapData.NumFrames())
print("mocapData.KeyFrameDuraction=", mocapData.KeyFrameDuraction())
print("mocapData.getCycleTime=", mocapData.getCycleTime())
print("mocapData.computeCycleOffset=", mocapData.computeCycleOffset())

qpi = quadrupedPoseInterpolator.QuadrupedPoseInterpolator()
stablePD = pd_controller_stable.PDControllerStable(p)

jointDirections = [1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1]
# mapping of laikago motions to quadruped
jointIds = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

jointOffsets = [0, -0.6, -1.8, 0, 0.6, 1.8]

timeStep = 1. / 50
cycleTime = mocapData.getCycleTime()
t = 0

for j in range(p.getNumJoints(env.quadruped)):
    info = p.getJointInfo(env.quadruped, j)
    print(info)

while 1:
    # get interpolated joint
    keyFrameDuration = mocapData.KeyFrameDuraction()
    cycleTime = mocapData.getCycleTime()
    cycleCount = mocapData.calcCycleCount(t, cycleTime)

    # print("cycleTime=",cycleTime)
    # print("cycleCount=",cycleCount)

    # print("cycles=",cycles)
    frameTime = t - cycleCount * cycleTime
    # print("frameTime=",frameTime)
    if (frameTime < 0):
        frameTime += cycleTime

    frame = int(frameTime / keyFrameDuration)
    frameNext = frame + 1
    if (frameNext >= mocapData.NumFrames()):
        frameNext = frame
    frameFraction = (frameTime - frame * keyFrameDuration) / (keyFrameDuration)
    print("frame=", frame)
    print("frameFraction=", frameFraction)
    frameData = mocapData._motion_data['Frames'][frame]
    frameDataNext = mocapData._motion_data['Frames'][frameNext]

    jointsStr, qdot = qpi.Slerp(frameFraction, frameData, frameDataNext, p)

    maxUpForce = 2.7

    basePos = [float(-jointsStr[0]), -float(jointsStr[1]), float(jointsStr[2])]
    baseOrn = [float(jointsStr[4]), float(jointsStr[5]), float(jointsStr[6]), float(jointsStr[3])]
    p.resetBasePositionAndOrientation(env.quadruped, basePos, cubeStartOrientation)

    # left front, for laikago 3, 4, 5 joints
    state_a_h = jointDirections[0] * float(jointsStr[3 + 7]) + jointOffsets[0]
    state_a_u = jointDirections[1] * float(jointsStr[4 + 7]) + jointOffsets[1]
    state_a_l = jointDirections[2] * float(jointsStr[5 + 7]) + jointOffsets[2]
    # right front, for laikago 0, 1, 2 joints
    state_b_h = jointDirections[3] * float(jointsStr[0 + 7]) + jointOffsets[3]
    state_b_u = jointDirections[4] * float(jointsStr[1 + 7]) + jointOffsets[4]
    state_b_l = jointDirections[5] * float(jointsStr[2 + 7]) + jointOffsets[5]

    state_c_h = state_b_h
    state_c_u = -state_b_u
    state_c_l = -state_b_l

    state_d_h = state_a_h
    state_d_u = -state_a_u
    state_d_l = -state_a_l

    p.resetJointState(env.quadruped, 0, state_a_h)
    p.resetJointState(env.quadruped, 1, state_a_u)
    p.resetJointState(env.quadruped, 2, state_a_l)
    p.resetJointState(env.quadruped, 3, state_b_h)
    p.resetJointState(env.quadruped, 4, state_b_u)
    p.resetJointState(env.quadruped, 5, state_b_l)
    p.resetJointState(env.quadruped, 6, state_c_h)
    p.resetJointState(env.quadruped, 7, state_c_u)
    p.resetJointState(env.quadruped, 8, state_c_l)
    p.resetJointState(env.quadruped, 9, state_d_h)
    p.resetJointState(env.quadruped, 10, state_d_u)
    p.resetJointState(env.quadruped, 11, state_d_l)

    t += timeStep
    time.sleep(timeStep)


print("run simulation")
# step through the simluation
for i in range(10000):
    p.stepSimulation()
    # p.getContactPoints(planeId, robot, )
    time.sleep(1./240.)
    #  robot.reset(reload_urdf=False)