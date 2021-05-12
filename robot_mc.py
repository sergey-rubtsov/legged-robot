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

leftLegFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftLeg', axes)))
leftFootFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftFoot', axes)))
leftToeBaseFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftToeBase', axes)))
rightUpLegFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightUpLeg', axes)))
rightLegFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightLeg', axes)))
rightFootFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightFoot', axes)))
rightToeBaseFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightToeBase', axes)))
spineFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:Spine', axes)))
spine1Frames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:Spine1', axes)))
spine2Frames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:Spine2', axes)))
leftShoulderFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftShoulder', axes)))
leftArmFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftArm', axes)))
leftHandFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftHand', axes)))
leftForeArmFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:LeftForeArm', axes)))

leftUpLegFrames = get_quaternions(mocap, 'mixamorig:LeftUpLeg', axes)
spineBvhFrames = get_quaternions(mocap, 'mixamorig:Spine', axes)
spine1BvhFrames = get_quaternions(mocap, 'mixamorig:Spine1', axes)
spine2BvhFrames = get_quaternions(mocap, 'mixamorig:Spine2', axes)
leftShoulderBvhFrames = get_quaternions(mocap, 'mixamorig:LeftShoulder', axes)
leftArmBvhFrames = get_quaternions(mocap, 'mixamorig:LeftArm', axes)
leftHandBvhFrames = get_quaternions(mocap, 'mixamorig:LeftHand', axes)
leftForeArmBvhFrames = get_quaternions(mocap, 'mixamorig:LeftForeArm', axes)
neckFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:Neck', axes)))
headFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:Head', axes)))
rightShoulderFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightShoulder', axes)))
rightArmFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightArm', axes)))
rightHandFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightHand', axes)))
rightForeArmFrames = list(map(convert_bvh_to_bullet, get_quaternions(mocap, 'mixamorig:RightForeArm', axes)))


def bullet_frame_for_bone(bone: str, frame_number: int, conventions: Quaternion):
    return convert_bvh_to_bullet(change_of_basis(convert_bvh_to_quaternion(get_quaternions(mocap, bone)[frame_number]),
                                                 conventions))


def bullet_frame_for_bone(bone: str, frame_number: int):
    return convert_bvh_to_bullet(get_quaternions(mocap, bone)[frame_number])


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

RightUpLegOrientation = Quaternion()
RightLegOrientation = Quaternion()
RightFootOrientation = Quaternion()
RightToeBaseOrientation = Quaternion()
LeftUpLegOrientation = Quaternion()

LeftLegOrientation = convert_xyz_euler_to_quaternion([-8.192083100766467e-07,
                                                      -0.0029844962991774082,
                                                      0.4437171220779419])
LeftFootOrientation = Quaternion()
LeftToeBaseOrientation = Quaternion()
SpineOrientation = Quaternion()
Spine1Orientation = Quaternion()
Spine2Orientation = Quaternion()
NeckOrientation = Quaternion()
HeadOrientation = Quaternion()
RightShoulderOrientation = Quaternion()
RightArmOrientation = Quaternion()
RightForeArmOrientation = Quaternion()
RightHandOrientation = Quaternion()
LeftShoulderOrientation = Quaternion()

LeftArmOrientation = convert_xyz_euler_to_quaternion([-0.27841538190841675,
                                                      -0.0007616957882419229,
                                                      -3.7203924421191914e-06])

LeftForeArmOrientation = Quaternion()
LeftHandOrientation = Quaternion()

disable_parent_collision()
p.setCollisionFilterPair(robot, robot, Spine2, LeftArm, 0)
p.setCollisionFilterPair(robot, robot, Spine2, RightArm, 0)

p.getCameraImage(320, 200)
frameReal = 0
numFrames = 30

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
    spine_rotation = convert_bvh_to_quaternion(spineBvhFrames[frame])
    # spine1_rotation = convert_bvh_to_quaternion(spine1BvhFrames[frame])
    # spine2_rotation = convert_bvh_to_quaternion(spine2BvhFrames[frame])
    # left_shoulder_rotation = convert_bvh_to_quaternion(leftShoulderBvhFrames[frame])
    # left_arm_rotation = convert_bvh_to_quaternion(leftArmBvhFrames[frame])
    # spine1_rotation = spine_rotation.rotate(convert_bvh_to_quaternion(spine1BvhFrames[frame]))
    # spine2_rotation = spine1_rotation.rotate(convert_bvh_to_quaternion(spine2BvhFrames[frame]))
    # left_shoulder_rotation = spine2_rotation.rotate(convert_bvh_to_quaternion(leftShoulderBvhFrames[frame]))

    # left_arm_rotation = convert_bvh_to_quaternion(leftArmBvhFrames[frame]).\
    #     __mul__(Quaternion(x)).\
    #     __mul__(convert_bvh_to_quaternion(leftArmBvhFrames[frame]).inverse)

    # left_up_leg_rotation = convert_bvh_to_quaternion(leftUpLegFrames[frame]).\
    #     __mul__(Quaternion(x=0, y=0.707, z=0.707, w=0)).\
    #     __mul__(convert_bvh_to_quaternion(leftUpLegFrames[frame]).inverse)
    # leftUpLegRotStart = convert_quaternion_to_bullet(left_up_leg_rotation)
    left_up_leg_rotation = Quaternion(x=0, y=-0.707, z=0.707, w=0).\
        __mul__(convert_bvh_to_quaternion(leftUpLegFrames[frame])).\
        __mul__(Quaternion(x=0, y=-0.707, z=0.707, w=0).inverse)
    leftUpLegRotStart = convert_quaternion_to_bullet(left_up_leg_rotation)
    leftUpLegRotStart2 = bullet_frame_for_bone('mixamorig:LeftUpLeg', frame, Quaternion(x=0, y=-0.707, z=0.707, w=0))

    left_arm_rotation = LeftArmOrientation.rotate(convert_bvh_to_quaternion(leftArmBvhFrames[frame]))
    # left_up_leg_rotation = LeftUpLegOrientation.rotate(convert_bvh_to_quaternion(leftUpLegFrames[frame]))
    # leftUpLegRotStart = convert_quaternion_to_bullet(left_up_leg_rotation)
    #
    # spineRotStart = convert_quaternion_to_bullet(spine_rotation)
    # spine1RotStart = convert_quaternion_to_bullet(spine1_rotation)
    # spine2RotStart = convert_quaternion_to_bullet(spine2_rotation)
    # leftShoulderRotStart = convert_quaternion_to_bullet(left_shoulder_rotation)
    leftArmRotStart = convert_quaternion_to_bullet(left_arm_rotation)
    spineRotStart = spineFrames[frame]
    spine1RotStart = spine1Frames[frame]
    spine2RotStart = spine2Frames[frame]
    leftShoulderRotStart = leftShoulderFrames[frame]
    # leftArmRotStart = leftArmFrames[frame]

    leftForeArmRotStart = leftForeArmFrames[frame]
    leftHandRotStart = leftHandFrames[frame]

    rightShoulderRotStart = rightShoulderFrames[frame]
    rightArmRotStart = rightArmFrames[frame]
    rightForeArmRotStart = rightForeArmFrames[frame]
    rightHandRotStart = rightHandFrames[frame]

    neckRotStart = neckFrames[frame]
    headRotStart = headFrames[frame]

    leftLegRotStart = leftLegFrames[frame]
    leftFootRotStart = leftFootFrames[frame]
    leftToeBaseRotStart = leftToeBaseFrames[frame]
    rightUpLegRotStart = rightUpLegFrames[frame]
    rightLegRotStart = rightLegFrames[frame]
    rightFootRotStart = rightFootFrames[frame]
    rightToeBaseRotStart = rightToeBaseFrames[frame]

    leftUpLegRotEnd = leftUpLegFrames[frameNext]

    leftUpLegRot = p.getQuaternionSlerp(leftUpLegRotStart, leftUpLegRotEnd, frameFraction)
    rightUpLegRotEnd = rightUpLegFrames[frameNext]
    rightUpLegRot = p.getQuaternionSlerp(rightUpLegRotStart, rightUpLegRotEnd, frameFraction)
    spineRotEnd = spineFrames[frameNext]
    spineRot = p.getQuaternionSlerp(spineRotStart, spineRotEnd, frameFraction)
    spine1RotEnd = spine1Frames[frameNext]
    spine1Rot = p.getQuaternionSlerp(spine1RotStart, spine1RotEnd, frameFraction)
    spine2RotEnd = spine2Frames[frameNext]
    spine2Rot = p.getQuaternionSlerp(spine2RotStart, spine2RotEnd, frameFraction)
    leftShoulderRotEnd = leftShoulderFrames[frameNext]
    leftShoulderRot = p.getQuaternionSlerp(leftShoulderRotStart, leftShoulderRotEnd, frameFraction)
    leftArmRotEnd = leftArmFrames[frameNext]
    # leftArmRot = p.getQuaternionSlerp(leftArmRotStart, leftArmRotEnd, frameFraction)
    leftHandRotEnd = leftHandFrames[frameNext]
    leftHandRot = p.getQuaternionSlerp(leftHandRotStart, leftHandRotEnd, frameFraction)
    neckRotEnd = neckFrames[frameNext]
    neckRot = p.getQuaternionSlerp(neckRotStart, neckRotEnd, frameFraction)
    rightShoulderRotEnd = rightShoulderFrames[frameNext]
    rightShoulderRot = p.getQuaternionSlerp(rightShoulderRotStart, rightShoulderRotEnd, frameFraction)
    rightArmRotEnd = rightArmFrames[frameNext]
    rightArmRot = p.getQuaternionSlerp(rightArmRotStart, rightArmRotEnd, frameFraction)
    rightHandRotEnd = rightHandFrames[frameNext]
    rightHandRot = p.getQuaternionSlerp(rightHandRotStart, rightHandRotEnd, frameFraction)
    leftLegRotEnd = leftLegFrames[frameNext]
    leftLegRot = p.getQuaternionSlerp(leftLegRotStart, leftLegRotEnd, frameFraction)
    leftFootRotEnd = leftFootFrames[frameNext]
    leftFootRot = p.getQuaternionSlerp(leftFootRotStart, leftFootRotEnd, frameFraction)
    leftToeBaseRotEnd = leftToeBaseFrames[frameNext]
    leftToeBaseRot = p.getQuaternionSlerp(leftToeBaseRotStart, leftToeBaseRotEnd, frameFraction)
    rightLegRotEnd = rightLegFrames[frameNext]
    rightLegRot = p.getQuaternionSlerp(rightLegRotStart, rightLegRotEnd, frameFraction)
    rightFootRotEnd = rightFootFrames[frameNext]
    rightFootRot = p.getQuaternionSlerp(rightFootRotStart, rightFootRotEnd, frameFraction)
    rightToeBaseRotEnd = rightToeBaseFrames[frameNext]
    rightToeBaseRot = p.getQuaternionSlerp(rightToeBaseRotStart, rightToeBaseRotEnd, frameFraction)
    leftForeArmRotEnd = leftForeArmFrames[frameNext]
    leftForeArmRot = p.getQuaternionSlerp(leftForeArmRotStart, leftForeArmRotEnd, frameFraction)
    rightForeArmRotEnd = rightForeArmFrames[frameNext]
    rightForeArmRot = p.getQuaternionSlerp(rightForeArmRotStart, rightForeArmRotEnd, frameFraction)
    headRotEnd = headFrames[frameNext]
    headRot = p.getQuaternionSlerp(headRotStart, headRotEnd, frameFraction)

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
    p.resetJointStateMultiDof(robot, LeftLeg, leftLegRotStart)
    p.resetJointStateMultiDof(robot, LeftFoot, leftFootRotStart)
    p.resetJointStateMultiDof(robot, LeftToeBase, leftToeBaseRotStart)
    p.resetJointStateMultiDof(robot, RightUpLeg, rightUpLegRotStart)
    p.resetJointStateMultiDof(robot, RightLeg, rightLegRotStart)
    p.resetJointStateMultiDof(robot, RightFoot, rightFootRotStart)
    p.resetJointStateMultiDof(robot, RightToeBase, rightToeBaseRotStart)


    p.stepSimulation()
    time.sleep(timeStep)
