import pybullet as p
from pyquaternion.quaternion import Quaternion
import json
import math
import time
import pybullet_data
import pybullet_data
import pathlib
from bvhtoolbox import BvhTree
from bvhtoolbox.bvhtransforms import get_affines, \
    get_euler_angles, \
    get_quaternions, \
    get_translations, \
    get_rotation_matrices, \
    get_motion_data, \
    set_motion_data, \
    prune

path = str(pathlib.Path(__file__).parent.absolute()) + "/kinematic/walk.bvh"


def convert_xyz_euler_to_quaternion(euler) -> Quaternion:
    q = p.getQuaternionFromEuler(euler)
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])


def convert_quaternion_to_bullet(q: Quaternion):
    return [q.x, q.y, q.z, q.w]


def convert_bvh_to_quaternion(bvh_q) -> Quaternion:
    return Quaternion(w=bvh_q[0], x=bvh_q[1], y=bvh_q[2], z=bvh_q[3])


def convert_bvh_to_bullet(bvh_q):
    return [bvh_q[1], bvh_q[2], bvh_q[3], bvh_q[0]]


with open(path) as file_handle:
    mocap = BvhTree(file_handle.read())
axes = 'rzxz'

leftLegFrames = get_quaternions(mocap, 'mixamorig:LeftLeg', axes)
leftFootFrames = get_quaternions(mocap, 'mixamorig:LeftFoot', axes)
leftToeBaseFrames = get_quaternions(mocap, 'mixamorig:LeftToeBase', axes)
rightUpLegFrames = get_quaternions(mocap, 'mixamorig:RightUpLeg', axes)
rightLegFrames = get_quaternions(mocap, 'mixamorig:RightLeg', axes)
rightFootFrames = get_quaternions(mocap, 'mixamorig:RightFoot', axes)
rightToeBaseFrames = get_quaternions(mocap, 'mixamorig:RightToeBase', axes)
spineFrames = get_quaternions(mocap, 'mixamorig:Spine', axes)
spine1Frames = get_quaternions(mocap, 'mixamorig:Spine1', axes)
spine2Frames = get_quaternions(mocap, 'mixamorig:Spine2', axes)
leftShoulderFrames = get_quaternions(mocap, 'mixamorig:LeftShoulder', axes)
leftArmFrames = get_quaternions(mocap, 'mixamorig:LeftArm', axes)
leftHandFrames = get_quaternions(mocap, 'mixamorig:LeftHand', axes)
leftForeArmFrames = get_quaternions(mocap, 'mixamorig:LeftForeArm', axes)
leftUpLegFrames = get_quaternions(mocap, 'mixamorig:LeftUpLeg', axes)
spineBvhFrames = get_quaternions(mocap, 'mixamorig:Spine', axes)
spine1BvhFrames = get_quaternions(mocap, 'mixamorig:Spine1', axes)
spine2BvhFrames = get_quaternions(mocap, 'mixamorig:Spine2', axes)
leftShoulderBvhFrames = get_quaternions(mocap, 'mixamorig:LeftShoulder', axes)
leftArmBvhFrames = get_quaternions(mocap, 'mixamorig:LeftArm', axes)
leftHandBvhFrames = get_quaternions(mocap, 'mixamorig:LeftHand', axes)
leftForeArmBvhFrames = get_quaternions(mocap, 'mixamorig:LeftForeArm', axes)
neckFrames = get_quaternions(mocap, 'mixamorig:Neck', axes)
headFrames = get_quaternions(mocap, 'mixamorig:Head', axes)
rightShoulderFrames = get_quaternions(mocap, 'mixamorig:RightShoulder', axes)
rightArmFrames = get_quaternions(mocap, 'mixamorig:RightArm', axes)
rightHandFrames = get_quaternions(mocap, 'mixamorig:RightHand', axes)
rightForeArmFrames = get_quaternions(mocap, 'mixamorig:RightForeArm', axes)


def bullet_frame_for_bone_transform(frames, frame_number: int, conventions: Quaternion):
    return convert_bvh_to_bullet(change_of_basis(convert_bvh_to_quaternion(frames[frame_number]),
                                                 conventions))


def bullet_frame_for_bone(frames, frame_number: int):
    return convert_bvh_to_bullet(frames[frame_number])


def change_of_basis(orientation: Quaternion, conventions: Quaternion) -> Quaternion:
    return conventions.__mul__(orientation).__mul__(conventions.inverse)


useGUI = True
if useGUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)
# p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
useZUp = True
useYUp = not useZUp

if useYUp:
    p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)

# p.resetDebugVisualizerCamera(cameraDistance=1,
#                              cameraYaw=0,
#                              cameraPitch=0,
#                              cameraTargetPosition=[0, 0, 0])

p.setTimeOut(1000000)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=30)

timeStep = 1. / 600.

p.setPhysicsEngineParameter(fixedTimeStep=timeStep)

jointTypes = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]
# flags = p.URDF_MAINTAIN_LINK_ORDER + p.URDF_USE_SELF_COLLISION
# robot = p.loadURDF("/urdf/robot.urdf",
#                    [0, 0, 0],
#                    useFixedBase=False,
#                    flags=flags)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.2]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
# flags = p.URDF_USE_SELF_COLLISION
flags = p.URDF_MAINTAIN_LINK_ORDER + p.URDF_USE_SELF_COLLISION
robot = p.loadURDF("/urdf/robot_no_hands.urdf", cubeStartPos, cubeStartOrientation, flags=flags)
# robot = p.loadURDF("/urdf/robot.urdf", [0, 0, 0],                  flags=flags)

humanoid_fix = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1.2], [0, 0, 0, 1])

p.resetBasePositionAndOrientation(robot, [0, 0, 0], [0, 0, 0, 1])


def disable_parent_collision():
    for num in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, num)
        if info[16] != -1:
            p.setCollisionFilterPair(robot, robot, info[0], info[16], 0)


for j in range(p.getNumJoints(robot)):
    ji = p.getJointInfo(robot, j)
    print(ji[1], "=", j, "  # type=", jointTypes[ji[2]])
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
    # print("joint[", j, "].name=", ji[1], "joint[", j, "].type=", jointTypes[ji[2]])

RightUpLeg = 0  # type= JOINT_SPHERICAL
RightLeg = 1  # type= JOINT_SPHERICAL
RightFoot = 2  # type= JOINT_SPHERICAL
RightToeBase = 3  # type= JOINT_SPHERICAL
LeftUpLeg = 4  # type= JOINT_SPHERICAL
LeftLeg = 5  # type= JOINT_SPHERICAL
LeftFoot = 6  # type= JOINT_SPHERICAL
LeftToeBase = 7  # type= JOINT_SPHERICAL
Spine = 8  # type= JOINT_FIXED
Spine1 = 9  # type= JOINT_SPHERICAL
Spine2 = 10  # type= JOINT_SPHERICAL
Neck = 11  # type= JOINT_SPHERICAL
Head = 12  # type= JOINT_SPHERICAL
RightShoulder = 13  # type= JOINT_SPHERICAL
RightArm = 14  # type= JOINT_SPHERICAL
RightForeArm = 15  # type= JOINT_SPHERICAL
RightHand = 16  # type= JOINT_FIXED
LeftShoulder = 17  # type= JOINT_SPHERICAL
LeftArm = 18  # type= JOINT_SPHERICAL
LeftForeArm = 19  # type= JOINT_SPHERICAL
LeftHand = 20  # type= JOINT_FIXED

LegOrientation = Quaternion(w=0, x=-0.707, y=0, z=0.707)           # x -90 z 180

FootOrientation = Quaternion(w=0.422618, x=-0.906308, y=0, z=0)     # x -130
ToeBaseOrientation = Quaternion(w=0.707, x=-0.707, y=0, z=0)        # x -90
SpineOrientation = Quaternion(w=0.999048, x=0.043619, y=0, z=0)     # x 5
Spine1Orientation = Quaternion(w=0.999048, x=0.043619, y=0, z=0)    # x 5
Spine2Orientation = Quaternion(w=0.997564, x=0.069756, y=0, z=0)    # x 8
NeckOrientation = Quaternion(w=0.996195, x=-0.087156, y=0, z=0)     # x -10
HeadOrientation = Quaternion(w=0.92388, x=-0.382683, y=0, z=0)      # x -45

# RightShoulderOrientation = Quaternion(w=-0.5, x=0.5, y=-0.5, z=0.5)   # x -90 z 270
# RightArmOrientation = Quaternion(w=-0.5, x=0.5, y=-0.5, z=0.5)
# RightForeArmOrientation = Quaternion(w=-0.5, x=0.5, y=-0.5, z=0.5)
# RightHandOrientation = Quaternion(w=-0.5, x=0.5, y=-0.5, z=0.5)
# LeftShoulderOrientation = Quaternion(w=0.5, x=-0.5, y=-0.5, z=0.5)   # x -90 z 90
# LeftArmOrientation = Quaternion(w=0.5, x=-0.5, y=-0.5, z=0.5)
# LeftForeArmOrientation = Quaternion(w=0.5, x=-0.5, y=-0.5, z=0.5)
# LeftHandOrientation = Quaternion(w=0.5, x=-0.5, y=-0.5, z=0.5)

RightShoulderOrientation = Quaternion(w=0.707, x=0, y=0.707, z=0)   # y 90
RightArmOrientation = Quaternion(w=0.707, x=0, y=0.707, z=0)
RightForeArmOrientation = Quaternion(w=0.707, x=0, y=0.707, z=0)
RightHandOrientation = Quaternion(w=0.707, x=0, y=0.707, z=0)
LeftShoulderOrientation = Quaternion(w=0.707, x=0, y=-0.707, z=0)   # y -90
LeftArmOrientation = Quaternion(w=0.707, x=0, y=-0.707, z=0)
LeftForeArmOrientation = Quaternion(w=0.707, x=0, y=-0.707, z=0)
LeftHandOrientation = Quaternion(w=0.707, x=0, y=-0.707, z=0)

disable_parent_collision()
p.setCollisionFilterPair(robot, robot, Spine2, LeftArm, 0)
p.setCollisionFilterPair(robot, robot, Spine2, RightArm, 0)

p.getCameraImage(320, 200)
frameReal = 0
numFrames = 5

while (p.isConnected()):
    i = 1 + 1
    erp = 0.2
    kpMotor = 0.2
    maxForce = 1000
    frameReal = frameReal + 1

    if frameReal >= numFrames:
        frameReal = 0
    kp = kpMotor

    frame = int(frameReal)
    frameNext = frame + 1
    if frameNext >= numFrames:
        frameNext = frame

    frameFraction = frameReal - frame
    spineRotStart = bullet_frame_for_bone(spineFrames, frame)
    spine1RotStart = bullet_frame_for_bone(spine1Frames, frame)
    spine2RotStart = bullet_frame_for_bone(spine2Frames, frame)

    leftShoulderRotStart = bullet_frame_for_bone_transform(leftShoulderFrames,
                                                           frame,
                                                           LeftArmOrientation)
    leftArmRotStart = bullet_frame_for_bone_transform(leftArmFrames,
                                                      frame,
                                                      LeftArmOrientation)
    leftForeArmRotStart = bullet_frame_for_bone_transform(leftForeArmFrames,
                                                          frame,
                                                          LeftArmOrientation)
    leftHandRotStart = bullet_frame_for_bone_transform(leftHandFrames,
                                                       frame,
                                                       LeftArmOrientation)

    rightShoulderRotStart = bullet_frame_for_bone_transform(rightShoulderFrames,
                                                            frame,
                                                            RightArmOrientation)
    rightArmRotStart = bullet_frame_for_bone_transform(rightArmFrames,
                                                       frame,
                                                       RightArmOrientation)
    rightForeArmRotStart = bullet_frame_for_bone_transform(rightForeArmFrames,
                                                           frame,
                                                           RightArmOrientation)
    rightHandRotStart = bullet_frame_for_bone_transform(rightHandFrames,
                                                        frame,
                                                        RightArmOrientation)

    neckRotStart = bullet_frame_for_bone(neckFrames, frame)
    headRotStart = bullet_frame_for_bone(headFrames, frame)

    leftUpLegRotStart = bullet_frame_for_bone_transform(leftUpLegFrames,
                                                        frame,
                                                        LegOrientation)
    leftLegRotStart = bullet_frame_for_bone(leftLegFrames, frame)
    leftFootRotStart = bullet_frame_for_bone_transform(leftFootFrames,
                                                       frame,
                                                       FootOrientation)
    leftToeBaseRotStart = bullet_frame_for_bone_transform(leftToeBaseFrames,
                                                          frame,
                                                          ToeBaseOrientation)

    rightUpLegRotStart = bullet_frame_for_bone_transform(rightUpLegFrames,
                                                         frame,
                                                         LegOrientation)
    rightLegRotStart = bullet_frame_for_bone(rightLegFrames, frame)
    rightFootRotStart = bullet_frame_for_bone_transform(rightFootFrames,
                                                        frame,
                                                        FootOrientation)
    rightToeBaseRotStart = bullet_frame_for_bone_transform(rightToeBaseFrames,
                                                           frame,
                                                           ToeBaseOrientation)

    # leftUpLegRotEnd = leftUpLegFrames[frameNext]
    # leftUpLegRot = p.getQuaternionSlerp(leftUpLegRotStart, leftUpLegRotEnd, frameFraction)
    # rightUpLegRotEnd = rightUpLegFrames[frameNext]
    # rightUpLegRot = p.getQuaternionSlerp(rightUpLegRotStart, rightUpLegRotEnd, frameFraction)
    # spineRotEnd = spineFrames[frameNext]
    # spineRot = p.getQuaternionSlerp(spineRotStart, spineRotEnd, frameFraction)
    # spine1RotEnd = spine1Frames[frameNext]
    # spine1Rot = p.getQuaternionSlerp(spine1RotStart, spine1RotEnd, frameFraction)
    # spine2RotEnd = spine2Frames[frameNext]
    # spine2Rot = p.getQuaternionSlerp(spine2RotStart, spine2RotEnd, frameFraction)
    # leftShoulderRotEnd = leftShoulderFrames[frameNext]
    # leftShoulderRot = p.getQuaternionSlerp(leftShoulderRotStart, leftShoulderRotEnd, frameFraction)
    # leftArmRotEnd = leftArmFrames[frameNext]
    # leftArmRot = p.getQuaternionSlerp(leftArmRotStart, leftArmRotEnd, frameFraction)
    # leftHandRotEnd = leftHandFrames[frameNext]
    # leftHandRot = p.getQuaternionSlerp(leftHandRotStart, leftHandRotEnd, frameFraction)
    # neckRotEnd = neckFrames[frameNext]
    # neckRot = p.getQuaternionSlerp(neckRotStart, neckRotEnd, frameFraction)
    # rightShoulderRotEnd = rightShoulderFrames[frameNext]
    # rightShoulderRot = p.getQuaternionSlerp(rightShoulderRotStart, rightShoulderRotEnd, frameFraction)
    # rightArmRotEnd = rightArmFrames[frameNext]
    # rightArmRot = p.getQuaternionSlerp(rightArmRotStart, rightArmRotEnd, frameFraction)
    # rightHandRotEnd = rightHandFrames[frameNext]
    # rightHandRot = p.getQuaternionSlerp(rightHandRotStart, rightHandRotEnd, frameFraction)
    # leftLegRotEnd = leftLegFrames[frameNext]
    # leftLegRot = p.getQuaternionSlerp(leftLegRotStart, leftLegRotEnd, frameFraction)
    # leftFootRotEnd = leftFootFrames[frameNext]
    # leftFootRot = p.getQuaternionSlerp(leftFootRotStart, leftFootRotEnd, frameFraction)
    # leftToeBaseRotEnd = leftToeBaseFrames[frameNext]
    # leftToeBaseRot = p.getQuaternionSlerp(leftToeBaseRotStart, leftToeBaseRotEnd, frameFraction)
    # rightLegRotEnd = rightLegFrames[frameNext]
    # rightLegRot = p.getQuaternionSlerp(rightLegRotStart, rightLegRotEnd, frameFraction)
    # rightFootRotEnd = rightFootFrames[frameNext]
    # rightFootRot = p.getQuaternionSlerp(rightFootRotStart, rightFootRotEnd, frameFraction)
    # rightToeBaseRotEnd = rightToeBaseFrames[frameNext]
    # rightToeBaseRot = p.getQuaternionSlerp(rightToeBaseRotStart, rightToeBaseRotEnd, frameFraction)
    # leftForeArmRotEnd = leftForeArmFrames[frameNext]
    # leftForeArmRot = p.getQuaternionSlerp(leftForeArmRotStart, leftForeArmRotEnd, frameFraction)
    # rightForeArmRotEnd = rightForeArmFrames[frameNext]
    # rightForeArmRot = p.getQuaternionSlerp(rightForeArmRotStart, rightForeArmRotEnd, frameFraction)
    # headRotEnd = headFrames[frameNext]
    # headRot = p.getQuaternionSlerp(headRotStart, headRotEnd, frameFraction)

    p.resetJointStateMultiDof(robot, Spine, spineRotStart)
    p.resetJointStateMultiDof(robot, Spine1, spine1RotStart)
    p.resetJointStateMultiDof(robot, Spine2, spine2RotStart)
    p.resetJointStateMultiDof(robot, RightShoulder, rightShoulderRotStart)
    p.resetJointStateMultiDof(robot, RightArm, rightArmRotStart)
    p.resetJointStateMultiDof(robot, RightForeArm, rightForeArmRotStart)
    p.resetJointStateMultiDof(robot, RightHand, rightHandRotStart)
    p.resetJointStateMultiDof(robot, LeftShoulder, leftShoulderRotStart)
    p.resetJointStateMultiDof(robot, LeftArm, leftArmRotStart)
    p.resetJointStateMultiDof(robot, LeftForeArm, leftForeArmRotStart)
    p.resetJointStateMultiDof(robot, LeftHand, leftHandRotStart)
    p.resetJointStateMultiDof(robot, Neck, neckRotStart)
    p.resetJointStateMultiDof(robot, Head, headRotStart)
    p.resetJointStateMultiDof(robot, LeftUpLeg, leftUpLegRotStart)
    # p.resetJointStateMultiDof(robot, LeftUpLeg, convert_bvh_to_bullet(leftUpLegFrames[0]))
    print("frame: ", frame, "bvh: ", leftUpLegFrames[frame], "result: ", leftUpLegRotStart)
    print("frame: ", frame,
          "bvh euler: ",
          p.getEulerFromQuaternion(convert_bvh_to_bullet(leftUpLegFrames[frame])),
          "result euler: ",
          p.getEulerFromQuaternion(leftUpLegRotStart))
    p.resetJointStateMultiDof(robot, LeftLeg, leftLegRotStart)
    p.resetJointStateMultiDof(robot, LeftFoot, leftFootRotStart)
    p.resetJointStateMultiDof(robot, LeftToeBase, leftToeBaseRotStart)
    p.resetJointStateMultiDof(robot, RightUpLeg, rightUpLegRotStart)
    p.resetJointStateMultiDof(robot, RightLeg, rightLegRotStart)
    p.resetJointStateMultiDof(robot, RightFoot, rightFootRotStart)
    p.resetJointStateMultiDof(robot, RightToeBase, rightToeBaseRotStart)

    p.stepSimulation()
    time.sleep(timeStep)
